import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_perceptron_driver() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("perceptron_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    perceptron_driver_pkg_dir = get_package_share_directory(
        "perceptron_driver"
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                perceptron_driver_pkg_dir, "launch/perceptron_driver_launch.py"
            )
        ),
        launch_arguments={"params_file": params_file}.items(),
    )


def launch_lidar_driver() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("perceptron_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    ldlidar_pkg_dir = get_package_share_directory("ldlidar_stl_ros2")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_pkg_dir, "launch/ld19.launch.py")
        ),
    )


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(launch_perceptron_driver())
    ld.add_action(launch_lidar_driver())

    return ld
