"""ZED + path overlay demo — one command visualization.

Real ZED camera (v5 wrapper) -> YOLO cones -> REAL stereo depth (falls back
to monocular when depth lookup is invalid) -> cone_mapper -> ft_planner ->
path + detections overlaid on the live image (OpenCV window and/or
/webcam/annotated_image topic).

Usage:
    ros2 launch webcam_path_demo zed_path_demo.launch.py          # window
    ros2 launch webcam_path_demo zed_path_demo.launch.py display:=false \
        save_debug_png:=/tmp/overlay.png                          # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("webcam_path_demo")
    demo_params = os.path.join(pkg_share, "config", "webcam_path_demo.yaml")
    planner_params = os.path.join(
        get_package_share_directory("planners"), "config", "ft_planner.yaml")
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py")

    display = LaunchConfiguration("display")
    save_debug_png = LaunchConfiguration("save_debug_png")

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={"camera_model": "zedm", "camera_name": "zed2i"}.items(),
    )

    webcam_node = Node(
        package="webcam_path_demo",
        executable="webcam_path_node",
        name="webcam_path_node",
        output="screen",
        parameters=[
            demo_params,
            {
                "source": "topic",
                "use_sim_time": False,
                "display": display,
                "save_debug_png": save_debug_png,
                "device": "0",
            },
        ],
    )

    cone_mapper_node = Node(
        package="cone_mapping",
        executable="cone_mapper_node",
        name="cone_mapper_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "input_topic": "slam/cone_detection",
                "output_topic": "/mapping/cone_map",
                "merge_radius": 0.5,
                "min_observations": 3,
                "max_age_sec": 600.0,
            }
        ],
    )

    ft_planner_node = Node(
        package="planners",
        executable="ft_planner_node",
        name="ft_planner_node",
        output="screen",
        parameters=[planner_params, {"use_sim_time": False}],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="track_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "1", "track", "base_link"],
    )

    mock_velocity = Node(
        package="webcam_path_demo",
        executable="mock_velocity",
        name="mock_velocity",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("display", default_value="true",
                              description="show the OpenCV overlay window"),
        DeclareLaunchArgument("save_debug_png", default_value="",
                              description="save the latest overlay to this path"),
        zed_node,
        webcam_node,
        cone_mapper_node,
        ft_planner_node,
        static_tf,
        mock_velocity,
    ])
