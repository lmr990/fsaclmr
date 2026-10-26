"""Launch eufs_sim and the QUTMS controller without perception, planning, CAN, or TF dependencies."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    track = LaunchConfiguration("track")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    eufs_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("qutms_eufs_bridge"), "launch", "eufs_control_sim.launch.py"])
        ),
        launch_arguments={"track": track, "gazebo_gui": gazebo_gui}.items(),
    )
    qutms_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("qutms_eufs_bridge"), "launch", "qutms_controller.launch.py"])
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("track", default_value="small_track"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            eufs_sim,
            qutms_controller,
        ]
    )
