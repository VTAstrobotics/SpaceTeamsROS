from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    """Generate launch description for running rosbridge_server and image_client."""

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    our_nav_share = get_package_share_directory('navigation')
    nav2_params = os.path.join(our_nav_shar, 'config', 'nav2_params.yaml')

    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge websocket server'
    )
    
    # Include rosbridge websocket launch
    rosbridge_launch = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen', # Optional: directs node output to the screen
            parameters=[
                {'port': 9090}, # Optional: specify the port, default is 9090
                # Add other rosbridge_server parameters here if needed
            ]
        )
    
    # Launch the image client node
    image_client_node = Node(
        package='space_teams_python',
        executable='image_client',
        name='image_client',
        output='screen'
    )

    # example of adding a node to the launch description
    # another_node = Node(
    #     package='your_package (probably space_teams_python)',
    #     executable='your_executable',
    #     name='your_node_name',
    #     output='screen'
    # )


    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    parameters={
          'frame_id':'base_link',
          'use_sim_time':True,
          'subscribe_depth':True,
          'use_action_for_goal':True,
          'Reg/Force3DoF':'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'false', # Use 2D occupancy
          'Grid/RangeMax':'5',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
          'wait_for_transform_duration': 1,
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
          'Grid/FrameId':'map'
    }

    return LaunchDescription([
        port_arg,
        rosbridge_launch,
        image_client_node, # , (make sure to separate with comma if adding a node)
        nav2_launch,
        # another_node
    ])