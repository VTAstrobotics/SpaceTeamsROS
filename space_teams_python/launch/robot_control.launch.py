from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os

def generate_launch_description():
    """Generate launch description for running rosbridge_server and image_client."""

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    our_nav_share = get_package_share_directory('navigation')
    nav2_params = os.path.join(our_nav_share, 'config', 'nav2_params.yaml')

    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    client_launch = Node( #change to our client code eventually
            package='space_teams_python',
            executable='example_client',
            output='screen', 
        )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_launch), 
                                 launch_arguments={
                                    'use_sim_time': 'true',
                                    'params_file': nav2_params,
                                }.items()
                            ),
        client_launch
    ])