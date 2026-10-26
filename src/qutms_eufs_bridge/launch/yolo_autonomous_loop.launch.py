"""Launch the full YOLO-based autonomous pipeline inside eufs_sim.

Combines:
  - eufs_launcher simulation (camera enabled) for the Gazebo world + car
  - yolo_cone_detection yolo_autonomous (YOLO detector + planner + controller)

This is a convenience wrapper so the GUI can launch the entire YOLO
autonomous stack with a single ``ros2 launch`` invocation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    track = LaunchConfiguration("track")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    tcone_yolo_path = LaunchConfiguration("tcone_yolo_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    planner = LaunchConfiguration("planner")

    # ---- EUFS simulation (camera enabled, lidar/laserscan disabled) ----
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("eufs_launcher"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "track": track,
            "gazebo_gui": gazebo_gui,
            "rviz": "true",
            "enable_camera": "true",
            "enable_lidar": "false",
            "enable_laserscan": "false",
        }.items(),
    )

    # ---- YOLO autonomous pipeline (detector + planner + controller) ----
    yolo_autonomous = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("yolo_cone_detection"),
                    "launch",
                    "yolo_autonomous.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "tcone_yolo_path": tcone_yolo_path,
            "planner": planner,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "track",
                default_value="small_oval",
                description="Track to load in eufs_sim",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="false",
                description="Launch the Gazebo 3D GUI window",
            ),
            DeclareLaunchArgument(
                "tcone_yolo_path",
                default_value=os.path.expanduser("~/qutms_ws/src/TCone-YOLO-main"),
                description="Absolute path to the TCone-YOLO repository root",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="urinay",
                description="Path planner: urinay (color-blind Delaunay) or ft (FaSTTUBe)",
            ),
            simulation,
            yolo_autonomous,
        ]
    )