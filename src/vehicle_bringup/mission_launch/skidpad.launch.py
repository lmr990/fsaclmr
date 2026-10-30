"""Real-car skidpad mission: ZED perception + skidpad planner + path tracker.

No Nav2 and no mission-specific controller: the existing Pure Pursuit tracker
consumes /planning/path_with_speeds from the skidpad planner. The ZED wrapper,
YOLO detector, cone mapper and static camera TF come from
zed_real_pipeline.launch.py.

The simulation acceptance uses qutms_eufs_bridge/yolo_autonomous_loop.launch.py
with planner:=skidpad instead; this launch is for the vehicle.
"""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    real_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                get_package_share_path("yolo_cone_detection")
                / "launch"
                / "zed_real_pipeline.launch.py"
            )
        ),
        launch_arguments={
            "camera_model": LaunchConfiguration("camera_model"),
            "camera_name": LaunchConfiguration("camera_name"),
            "cam_x": LaunchConfiguration("cam_x"),
            "cam_y": LaunchConfiguration("cam_y"),
            "cam_z": LaunchConfiguration("cam_z"),
            "cam_pitch": LaunchConfiguration("cam_pitch"),
            "publish_track_tf": LaunchConfiguration("publish_track_tf"),
        }.items(),
    )

    skidpad_node = Node(
        package="planners",
        executable="skidpad_planner_node",
        name="skidpad_planner_node",
        output="screen",
        parameters=[
            str(get_package_share_path("planners") / "config" / "skidpad.yaml"),
            {"use_sim_time": False},
        ],
    )

    path_tracker = Node(
        package="controllers",
        executable="path_tracker_node",
        name="path_tracker_node",
        output="screen",
        parameters=[
            str(get_package_share_path("controllers") / "config" / "control.yaml"),
            {
                "use_sim_time": False,
                # Pure pursuit already follows the path curvature; adding
                # feedforward double counts it on the constant-radius circles.
                "curvature_ff_gain": 0.0,
            },
        ],
    )

    safety_watchdog = Node(
        package="controllers",
        executable="safety_watchdog_node",
        name="safety_watchdog_node",
        output="screen",
        parameters=[
            str(get_package_share_path("controllers") / "config" / "control.yaml"),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_model", default_value="zedm"),
            DeclareLaunchArgument("camera_name", default_value="zed2i"),
            DeclareLaunchArgument("cam_x", default_value="-0.370"),
            DeclareLaunchArgument("cam_y", default_value="0.0"),
            DeclareLaunchArgument("cam_z", default_value="0.697"),
            DeclareLaunchArgument("cam_pitch", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_track_tf",
                default_value="true",
                description="false when localization already owns track->base_link",
            ),
            real_pipeline,
            skidpad_node,
            path_tracker,
            safety_watchdog,
        ]
    )
