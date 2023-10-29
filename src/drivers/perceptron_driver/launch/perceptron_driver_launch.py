import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = get_package_share_directory("perceptron_driver")
    default_param_file = os.path.join(pkg_share, "config", "params.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_param_file,
        description="Path to the ROS2 parameters file to use.",
    )

    params_file = LaunchConfiguration("params_file")

    perceptron_driver_node = Node(
        package="perceptron_driver",
        executable="driver_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([params_file_arg, perceptron_driver_node])
