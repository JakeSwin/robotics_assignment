import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

os.environ["TURTLEBOT3_MODEL"] = "waffle"

def generate_launch_description():
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')]),
        launch_arguments={
            "use_sim_time": "true",
            "map": os.path.join(
            get_package_share_directory('colour_follower'), 'config/assignment_map.yaml'),
            "params_file": os.path.join(
            get_package_share_directory('colour_follower'), 'config/waffle.yaml')
        }.items(),
    )
    change_model_type = ExecuteProcess(
        cmd=[[
            "ros2 param set ",
            "/amcl ",
            "robot_model_type ",
            "nav2_amcl::DifferentialMotionModel"
        ]],
        shell=True
    )

    return LaunchDescription([
        navigation,
        change_model_type,
    ])