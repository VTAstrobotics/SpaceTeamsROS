// Copyright (c) 2014, Worcester Polytechnic Institute
// Copyright (c) 2024, The Robot Web Tools Contributors
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "web_video_server/libav_streamer.hpp"
#include "async_web_server_cpp/http_reply.hpp"

// https://stackoverflow.com/questions/46884682/error-in-building-opencv-with-ffmpeg
#define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER

namespace web_video_server
{

LibavStreamer::LibavStreamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection, rclcpp::Node::SharedPtr node,
  const std::string & format_name, const std::string & codec_name,
  const std::string & content_type)
: ImageTransportImageStreamer(request, connection, node), format_context_(0), codec_(0),
  codec_context_(0), video_stream_(0), opt_(0), frame_(0), sws_context_(0),
  first_image_received_(false), first_image_time_(), format_name_(format_name),
  codec_name_(codec_name), content_type_(content_type), io_buffer_(0)
{
  bitrate_ = request.get_query_param_value_or_default<int>("bitrate", 100000);
  qmin_ = request.get_query_param_value_or_default<int>("qmin", 10);
  qmax_ = request.get_query_param_value_or_default<int>("qmax", 42);
  gop_ = request.get_query_param_value_or_default<int>("gop", 25);
}

LibavStreamer::~LibavStreamer()
{
  if (codec_context_) {
    avcodec_free_context(&codec_context_);
  }
  if (frame_) {
    av_frame_free(&frame_);
  }
  if (io_buffer_) {
    delete io_buffer_;
  }
  if (format_context_) {
    if (format_context_->pb) {
      av_free(format_context_->pb);
    }
    avformat_free_context(format_context_);
  }
  if (sws_context_) {
    sws_freeContext(sws_context_);
  }
}

// output callback for ffmpeg IO context
#if LIBAVFORMAT_VERSION_MAJOR < 61
static int dispatch_output_packet(void * opaque, uint8_t * buffer, int buffer_size)
#else
static int dispatch_output_packet(void * opaque, const uint8_t * buffer, int buffer_size)
#endif
{
  async_web_server_cpp::HttpConnectionPtr connection =
    *((async_web_server_cpp::HttpConnectionPtr *) opaque);
  std::vector<uint8_t> encoded_frame;
  encoded_frame.assign(buffer, buffer + buffer_size);
  connection->write_and_clear(encoded_frame);
  return 0;
}

void LibavStreamer::initialize(const cv::Mat & /* img */)
{
  // Load format
  format_context_ = avformat_alloc_context();
  if (!format_context_) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error allocating ffmpeg format context");
  }

  format_context_->oformat = av_guess_format(format_name_.c_str(), NULL, NULL);
  if (!format_context_->oformat) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error looking up output format");
  }

  // Set up custom IO callback.
  size_t io_buffer_size = 3 * 1024;    // 3M seen elsewhere and adjudged good
  io_buffer_ = new unsigned char[io_buffer_size];
  AVIOContext * io_ctx = avio_alloc_context(
    io_buffer_, io_buffer_size, AVIO_FLAG_WRITE,
    &connection_, NULL, dispatch_output_packet, NULL);
  if (!io_ctx) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error setting up IO context");
  }
  io_ctx->seekable = 0;                       // no seeking, it's a stream
  format_context_->pb = io_ctx;
  format_context_->max_interleave_delta = 0;

  // Load codec
  if (codec_name_.empty()) {  // use default codec if none specified
    codec_ = avcodec_find_encoder(format_context_->oformat->video_codec);
  } else {
    codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
  }
  if (!codec_) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error looking up codec");
  }

  video_stream_ = avformat_new_stream(format_context_, codec_);
  if (!video_stream_) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error creating video stream");
  }

  codec_context_ = avcodec_alloc_context3(codec_);
  if (!codec_context_) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error allocating codec context");
  }

  // Set options on codec_context_ BEFORE opening encoder
  codec_context_->codec_id = codec_->id;
  codec_context_->bit_rate = bitrate_;

  codec_context_->width = output_width_;
  codec_context_->height = output_height_;
  codec_context_->delay = 0;

  // Set stream timebase (container) — leave in ms if you prefer (we rescale packets)
  video_stream_->time_base.num = 1;
  video_stream_->time_base.den = 1000;

  // Set encoder timebase based on desired FPS
  // <-- ADAPT: ensure this member exists in your class if you want a non-1 timebase
  codec_context_->time_base.num = 1;
  codec_context_->time_base.den = 1; // keep same semantics as your current code

  codec_context_->gop_size = gop_;
  codec_context_->pix_fmt = AV_PIX_FMT_YUV420P; // keep as you had
  codec_context_->max_b_frames = 0;

  // Quality settings
  codec_context_->qmin = qmin_;
  codec_context_->qmax = qmax_;

  codec_context_->flags |= AV_CODEC_FLAG_LOW_DELAY;

  // Allow subclass to set private options (x264 preset/crf etc.)
  initializeEncoder();

  // If the container format expects global headers, request them from the encoder
  if (format_context_->oformat && (format_context_->oformat->flags & AVFMT_GLOBALHEADER)) {
    codec_context_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
  }

  // Open the codec FIRST so encoder can populate extradata (SPS/PPS) into codec_context_
  if (avcodec_open2(codec_context_, codec_, NULL) < 0) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Could not open video codec");
  }

  // Copy codec params into the stream (this may copy extradata if the encoder filled it)
  if (avcodec_parameters_from_context(video_stream_->codecpar, codec_context_) < 0) {
    throw std::runtime_error("Failed to copy codec parameters to stream");
  }

  // Buffer to hold any probe packets (if needed)
  std::vector<AVPacket *> buffered_pkts;

  // If extradata is missing, try to force the encoder to emit SPS/PPS by encoding a probe frame
  if (!(video_stream_->codecpar->extradata && video_stream_->codecpar->extradata_size > 0)) {
    RCLCPP_WARN(node_->get_logger(),
                "No extradata found in codecpar — attempting to force encoder to emit SPS/PPS");

    // Prepare a simple black probe frame matching codec dimensions and pix_fmt
    AVFrame *probe_frame = av_frame_alloc();
    if (!probe_frame) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to alloc probe frame");
    } else {
      probe_frame->format = codec_context_->pix_fmt;
      probe_frame->width  = codec_context_->width;
      probe_frame->height = codec_context_->height;
      if (av_frame_get_buffer(probe_frame, 32) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to allocate probe frame buffer");
      } else {
        // zero the frame (black)
        if (av_frame_make_writable(probe_frame) == 0) {
          for (int i = 0; i < AV_NUM_DATA_POINTERS; ++i) {
            if (probe_frame->data[i] && probe_frame->linesize[i] > 0) {
              // write height lines worth of zeros for each plane
              memset(probe_frame->data[i], 0x00,
                     static_cast<size_t>(probe_frame->linesize[i]) * codec_context_->height);
            }
          }
        }

        // Give it a pts (in codec timebase)
        static int64_t probe_pts = 0;
        probe_frame->pts = probe_pts++;

        // Send frame and collect any output packets (SPS/PPS often emitted as side-data or first packet)
        int ret = avcodec_send_frame(codec_context_, probe_frame);
        if (ret < 0) {
          char errbuf[AV_ERROR_MAX_STRING_SIZE];
          av_strerror(ret, errbuf, sizeof(errbuf));
          RCLCPP_ERROR(node_->get_logger(), "avcodec_send_frame (probe) failed: %s", errbuf);
        } else {
          while (true) {
            AVPacket *pkt = av_packet_alloc();
            if (!pkt) break;
            ret = avcodec_receive_packet(codec_context_, pkt);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
              av_packet_free(&pkt);
              break;
            } else if (ret < 0) {
              char errbuf[AV_ERROR_MAX_STRING_SIZE];
              av_strerror(ret, errbuf, sizeof(errbuf));
              RCLCPP_ERROR(node_->get_logger(), "avcodec_receive_packet (probe) error: %s", errbuf);
              av_packet_free(&pkt);
              break;
            } else {
              // keep packet for writing after header
              buffered_pkts.push_back(pkt);
            }
          }
        }
      }
      av_frame_free(&probe_frame);
    }

    // Copy codec params again (encoder may have populated extradata during probe)
    if (avcodec_parameters_from_context(video_stream_->codecpar, codec_context_) < 0) {
      char errbuf[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(AVERROR_UNKNOWN, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(node_->get_logger(), "Failed to copy codec parameters to stream after probe: %s", errbuf);
    } else {
      if (video_stream_->codecpar->extradata && video_stream_->codecpar->extradata_size > 0) {
        RCLCPP_INFO(node_->get_logger(),
                    "Extradata created after probe (size=%d bytes)",
                    video_stream_->codecpar->extradata_size);
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "Still no extradata after probe; buffered packets: %zu",
                    buffered_pkts.size());
      }
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "H264 extradata present (size=%d bytes)", video_stream_->codecpar->extradata_size);
  }

  // Allocate frame buffers AFTER codec is opened
  frame_ = av_frame_alloc();
  if (!frame_) {
    throw std::runtime_error("Failed to allocate frame");
  }

  if (av_image_alloc(
      frame_->data, frame_->linesize, output_width_, output_height_,
      codec_context_->pix_fmt, 1) < 0)
  {
    av_frame_free(&frame_);
    throw std::runtime_error("Failed to allocate image buffers for frame");
  }

  frame_->width = output_width_;
  frame_->height = output_height_;
  frame_->format = codec_context_->pix_fmt;

  // define meta data
  av_dict_set(&format_context_->metadata, "author", "ROS web_video_server", 0);
  av_dict_set(&format_context_->metadata, "title", topic_.c_str(), 0);

  // Send response headers (HTTP)
  async_web_server_cpp::HttpReply::builder(async_web_server_cpp::HttpReply::ok)
  .header("Connection", "close")
  .header("Server", "web_video_server")
  .header(
    "Cache-Control",
    "no-cache, no-store, must-revalidate, pre-check=0, post-check=0, max-age=0")
  .header("Pragma", "no-cache")
  .header("Expires", "0")
  .header("Max-Age", "0")
  .header("Trailer", "Expires")
  .header("Content-type", content_type_)
  .header("Access-Control-Allow-Origin", "*")
  .write(connection_);

  // Send video stream header NOW that codecpar hopefully contains extradata.
  if (avformat_write_header(format_context_, &opt_) < 0) {
    async_web_server_cpp::HttpReply::stock_reply(
      async_web_server_cpp::HttpReply::internal_server_error)(request_, connection_, NULL, NULL);
    throw std::runtime_error("Error opening dynamic buffer (avformat_write_header failed)");
  }

  // If we buffered any probe packets, write them now (rescale timestamps first)
  for (AVPacket *pkt : buffered_pkts) {
    av_packet_rescale_ts(pkt, codec_context_->time_base, video_stream_->time_base);
    pkt->stream_index = video_stream_->index;
    int wret = av_interleaved_write_frame(format_context_, pkt);
    if (wret < 0) {
      char errbuf[AV_ERROR_MAX_STRING_SIZE];
      av_strerror(wret, errbuf, sizeof(errbuf));
      RCLCPP_ERROR(node_->get_logger(), "Failed to write buffered probe packet: %s", errbuf);
    }
    av_packet_free(&pkt);
  }
  buffered_pkts.clear();
}


void LibavStreamer::initializeEncoder()
{
}

void LibavStreamer::sendImage(
  const cv::Mat & img,
  const std::chrono::steady_clock::time_point & time)
{
  std::scoped_lock lock(encode_mutex_);
  if (!first_image_received_) {
    first_image_received_ = true;
    first_image_time_ = time;
  }

  AVPixelFormat input_coding_format = AV_PIX_FMT_BGR24;

  AVFrame * raw_frame = av_frame_alloc();
  av_image_fill_arrays(
    raw_frame->data, raw_frame->linesize,
    img.data, input_coding_format, output_width_, output_height_, 1);

  // Convert from opencv to libav
  if (!sws_context_) {
    static int sws_flags = SWS_BICUBIC;
    sws_context_ = sws_getContext(
      output_width_, output_height_, input_coding_format, output_width_,
      output_height_, codec_context_->pix_fmt, sws_flags, NULL, NULL, NULL);
    if (!sws_context_) {
      throw std::runtime_error("Could not initialize the conversion context");
    }
  }


  int ret = sws_scale(
    sws_context_,
    (const uint8_t * const *)raw_frame->data, raw_frame->linesize, 0,
    output_height_, frame_->data, frame_->linesize);

  av_frame_free(&raw_frame);

  // Encode the frame
  AVPacket * pkt = av_packet_alloc();

  ret = avcodec_send_frame(codec_context_, frame_);
  if (ret == AVERROR_EOF) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "avcodec_send_frame() encoder flushed\n");
  } else if (ret == AVERROR(EAGAIN)) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "avcodec_send_frame() need output read out\n");
  }
  if (ret < 0) {
    throw std::runtime_error("Error encoding video frame");
  }

  ret = avcodec_receive_packet(codec_context_, pkt);
  bool got_packet = pkt->size > 0;
  if (ret == AVERROR_EOF) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "avcodec_receive_packet() encoder flushed\n");
  } else if (ret == AVERROR(EAGAIN)) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "avcodec_receive_packet() needs more input\n");
    got_packet = false;
  }

  if (got_packet) {
    double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(
      time - first_image_time_).count();
    // Encode video at 1/0.95 to minimize delay
    pkt->pts = (int64_t)(seconds / av_q2d(video_stream_->time_base) * 0.95);
    if (pkt->pts <= 0) {
      pkt->pts = 1;
    }
    pkt->dts = pkt->pts;

    if (pkt->flags & AV_PKT_FLAG_KEY) {
      pkt->flags |= AV_PKT_FLAG_KEY;
    }

    pkt->stream_index = video_stream_->index;

    if (av_write_frame(format_context_, pkt)) {
      throw std::runtime_error("Error when writing frame");
    }
  }

  av_packet_unref(pkt);
}

LibavStreamerType::LibavStreamerType(
  const std::string & format_name, const std::string & codec_name,
  const std::string & content_type)
: format_name_(format_name), codec_name_(codec_name), content_type_(content_type)
{
}

std::shared_ptr<ImageStreamer> LibavStreamerType::create_streamer(
  const async_web_server_cpp::HttpRequest & request,
  async_web_server_cpp::HttpConnectionPtr connection,
  rclcpp::Node::SharedPtr node)
{
  return std::make_shared<LibavStreamer>(
    request, connection, node, format_name_, codec_name_,
    content_type_);
}

std::string LibavStreamerType::create_viewer(const async_web_server_cpp::HttpRequest & request)
{
  std::stringstream ss;
  ss << "<video src=\"/stream?";
  ss << request.query;
  ss << "\" autoplay=\"true\" preload=\"none\"></video>";
  return ss.str();
}

}  // namespace web_video_server
