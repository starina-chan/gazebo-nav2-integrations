import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file", default="map_to_gazebo.yaml")
    export_dir = LaunchConfiguration("export_dir")
    config_path = [
        os.path.join(get_package_share_directory("gazebo_nav2_integrations"), "config"),
        "/",
        params_file,
    ]

    export_dir_arg = DeclareLaunchArgument(
        "export_dir",
        default_value=os.path.abspath("."),
        description="Dir map_to_gazebo writes stl/sdf/world files to",
    )
    map_to_gazebo_node = Node(
        package="gazebo_nav2_integrations",
        executable="map_to_gazebo",
        name="map_to_gazebo",
        output="screen",
        parameters=[config_path, {"export_dir": export_dir}],
    )

    return LaunchDescription([export_dir_arg, map_to_gazebo_node])
