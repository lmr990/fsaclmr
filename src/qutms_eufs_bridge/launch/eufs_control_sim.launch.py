"""Launch only eufs_sim for direct low-speed Ackermann command testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    track = LaunchConfiguration("track")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    enable_camera = LaunchConfiguration("enable_camera")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("eufs_launcher"), "launch", "simulation.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "track": track,
            "gazebo_gui": gazebo_gui,
            "rviz": "true",
            "enable_camera": enable_camera,
            "enable_lidar": "false",
            "enable_laserscan": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("track", default_value="small_track"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="false"),
            simulation,
        ]
    )