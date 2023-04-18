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
            get_package_share_directory('colour_follower'), 'navigation2.launch.py')]),
        launch_arguments={
            "use_sim_time": "true",
            "map": os.path.join(
            get_package_share_directory('colour_follower'), 'config/assignment_map.yaml'),
            "params_file": os.path.join(
            get_package_share_directory('colour_follower'), 'config/waffle.yaml')
        }.items(),
    )
    # My stuff
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('colour_follower'), '/twist_mux_launch.py'])
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
        twist_mux,
        navigation,
        change_model_type,
    ])