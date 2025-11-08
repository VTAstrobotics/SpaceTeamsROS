from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node

class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/depth/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5

    def image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = depth_image.shape
        points = []

        for v in range(h):
            for u in range(w):
                z = depth_image[v, u]
                if z == 0 or np.isnan(z):
                    continue
                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy
                points.append([x, y, z])

        header = msg.header
        pc2 = point_cloud2.create_cloud_xyz32(header, points)
        self.pub.publish(pc2)
        self.get_logger().info(f'Published PointCloud with {len(points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
