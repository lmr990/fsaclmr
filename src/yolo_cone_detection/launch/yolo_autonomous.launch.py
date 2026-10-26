"""Launch the full YOLO-based autonomous pipeline (upgraded stack).

Includes:
  - yolo_detector_node  : camera -> YOLO -> slam/cone_detection (track frame)
  - cone_mapper_node    : slam/cone_detection -> /mapping/cone_map (weighted merge)
  - ft_planner_node     : /mapping/cone_map -> /planning/path_with_speeds
                          (midline + curvature + three-pass speed profile)
  - path_tracker_node   : path_with_speeds -> /control/driving_command
                          (adaptive-lookahead PP + curvature FF + steering PID)
  - safety_watchdog_node: wall-clock staleness failsafe -> stop command

NOTE: The Gazebo sim must be launched separately with enable_camera:=true.
Legacy nodes (path_follower_node / vel_to_ackermann_node) remain available
in the controllers package but are no longer part of this pipeline.
"""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    tcone_yolo_path = LaunchConfiguration("tcone_yolo_path")

    # ---- YOLO detector ----
    yolo_params_file = os.path.join(
        get_package_share_path("yolo_cone_detection"), "config", "yolo_detector.yaml"
    )

    yolo_node = Node(
        package="yolo_cone_detection",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            yolo_params_file,
            {
                "use_sim_time": use_sim_time,
                "tcone_yolo_path": tcone_yolo_path,
            },
        ],
    )

    # ---- Cone mapper (observation-weighted map accumulation) ----
    cone_mapper_node = Node(
        package="cone_mapping",
        executable="cone_mapper_node",
        name="cone_mapper_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "slam/cone_detection",
                "output_topic": "/mapping/cone_map",
                "merge_radius": 0.5,
                "min_observations": 3,
                "max_age_sec": 120.0,
            }
        ],
    )

    # ---- Planner (FaSTTUBe boundary extraction + midline + speed profile) ----
    planner_node = Node(
        package="planners",
        executable="ft_planner_node",
        name="ft_planner_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "ft_planner.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---- Path tracker (adaptive-lookahead PP + curvature FF + PID) ----
    path_tracker_node = Node(
        package="controllers",
        executable="path_tracker_node",
        name="path_tracker_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_path("controllers"), "config", "control.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---- Safety watchdog (wall-clock staleness failsafe) ----
    safety_watchdog_node = Node(
        package="controllers",
        executable="safety_watchdog_node",
        name="safety_watchdog_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_path("controllers"), "config", "control.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock",
            ),
            DeclareLaunchArgument(
                "tcone_yolo_path",
                default_value="/home/user/qutms_ws/src/TCone-YOLO-main",
                description="Absolute path to TCone-YOLO repository root",
            ),
            yolo_node,
            cone_mapper_node,
            planner_node,
            path_tracker_node,
            safety_watchdog_node,
        ]
    )
