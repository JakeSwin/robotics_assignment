import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('colour_follower'), '/twist_mux_launch.py'])
    )
    # wanderer = Node(
    #     package='colour_follower',
    #     executable='wanderer',
    #     output='screen'
    # )
    drive_forward = Node(
        package='colour_follower',
        executable='drive_forward',
        output='screen'
    )
    recovery = Node(
        package='colour_follower',
        executable='recovery',
        output='screen'
    )
    colour_chaser = Node(
        package='colour_follower',
        executable='colour_chaser',
        output='screen'
    )

    return LaunchDescription([
        twist_mux,
        drive_forward,
        recovery,
        colour_chaser
        # wanderer
    ])