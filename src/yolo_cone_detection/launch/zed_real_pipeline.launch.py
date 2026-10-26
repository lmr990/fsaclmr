"""Real ZED camera (ZED-M) cone detection pipeline.

Brings up the ZED ROS 2 wrapper (v5 topic naming) plus the project's YOLO
detector and cone mapper, with topic overrides bridging the wrapper v5 names
to what the detection stack consumes:

    wrapper v5                         detection stack expects
    /zed2i/zed_node/rgb/color/rect/image      /zed2i/zed_node/rgb/image_rect_color
    /zed2i/zed_node/depth/depth_registered    (unchanged)

Usage:
    ros2 launch yolo_cone_detection zed_real_pipeline.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("yolo_cone_detection")
    params = os.path.join(pkg_share, "config", "yolo_detector.yaml")
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
    )

    camera_name = LaunchConfiguration("camera_name")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": "zedm",
            "camera_name": camera_name,
        }.items(),
    )

    yolo_node = Node(
        package="yolo_cone_detection",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            params,
            {
                "use_sim_time": False,
                "image_topic": image_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
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

    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="zed2i",
                              description="namespace; zed2i keeps config parity with sim"),
        DeclareLaunchArgument("image_topic", default_value="/zed2i/zed_node/rgb/color/rect/image",
                              description="wrapper v5 RGB topic"),
        DeclareLaunchArgument("depth_topic", default_value="/zed2i/zed_node/depth/depth_registered"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/zed2i/zed_node/rgb/color/rect/image/camera_info"),
        zed_node,
        yolo_node,
        cone_mapper_node,
    ])
