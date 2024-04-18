import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_navigation_stack() -> IncludeLaunchDescription:
    navigation_pkg_dir = get_package_share_directory("nav2_bringup")
    params_file = os.path.join(
        get_package_share_directory("perceptron_bringup"),
        "config",
        "params.yaml",
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params_file,
        }.items(),
    )


def generate_slam_toolbox_launch() -> IncludeLaunchDescription:
    pkg_dir = get_package_share_directory("unitree_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    slam_toolbox_pkg_dir = get_package_share_directory("slam_toolbox")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_dir, "launch/online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": params_file,
            "mode": "localization",
            "use_sim_time": "false",
        }.items(),
    )


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(launch_navigation_stack())
    # ld.add_action(generate_slam_toolbox_launch())

    return ld
