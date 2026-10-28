"""Real ZED camera (ZED-M) cone detection pipeline.

Brings up the ZED ROS 2 wrapper (v5 topic naming) plus the project's YOLO
detector and cone mapper, with topic overrides bridging the wrapper v5 names
to what the detection stack consumes:

    wrapper v5                         detection stack expects
    /zed2i/zed_node/rgb/color/rect/image      /zed2i/zed_node/rgb/image_rect_color
    /zed2i/zed_node/depth/depth_registered    (unchanged)

TF: the wrapper's visual-odometry TF is disabled. ``zed2i_camera_link`` may
only have ONE parent, so ``odom -> zed2i_camera_link`` (wrapper) and
``base_link -> zed2i_camera_link`` (mount) cannot coexist -- tf2 silently
drops the second tree. This launch owns the static camera mount instead:

    track -> base_link -> zed2i_camera_link -> ..._left_camera_frame_optical

The detector needs ``track <- optical`` to place cones in ``track``; without
this chain it skips every frame ("TF unavailable"). On a car running
localization, set ``publish_track_tf:=false`` and let localization publish
``track -> base_link``.

Usage:
    ros2 launch yolo_cone_detection zed_real_pipeline.launch.py
    ros2 launch yolo_cone_detection zed_real_pipeline.launch.py \
        cam_z:=0.35 cam_pitch:=-0.26          # bench measurement
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("yolo_cone_detection")
    params = os.path.join(pkg_share, "config", "yolo_detector.yaml")
    zed_launch = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
    )

    camera_name = LaunchConfiguration("camera_name")
    camera_model = LaunchConfiguration("camera_model")
    image_topic = LaunchConfiguration("image_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    base_frame = LaunchConfiguration("base_frame")
    track_frame = LaunchConfiguration("track_frame")
    cam_x = LaunchConfiguration("cam_x")
    cam_y = LaunchConfiguration("cam_y")
    cam_z = LaunchConfiguration("cam_z")
    cam_roll = LaunchConfiguration("cam_roll")
    cam_pitch = LaunchConfiguration("cam_pitch")
    cam_yaw = LaunchConfiguration("cam_yaw")

    camera_link = PythonExpression(["'", camera_name, "' + '_camera_link'"])

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": camera_model,
            "camera_name": camera_name,
            # The mount below is the only parent of <camera>_camera_link.
            "publish_tf": "false",
            "publish_map_tf": "false",
        }.items(),
    )

    # Car at the origin of `track` (display/bench mode). Localization
    # replaces this on the car: publish_track_tf:=false.
    track_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="track_to_base_link",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", track_frame, "--child-frame-id", base_frame,
        ],
        condition=IfCondition(LaunchConfiguration("publish_track_tf")),
    )

    # Measured camera mount w.r.t. base_link (defaults: sim URDF, camera
    # 0.900 m above chassis, base_link 0.2032 m below chassis).
    mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_link",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        arguments=[
            "--x", cam_x, "--y", cam_y, "--z", cam_z,
            "--roll", cam_roll, "--pitch", cam_pitch, "--yaw", cam_yaw,
            "--frame-id", base_frame, "--child-frame-id", camera_link,
        ],
    )

    yolo_node = Node(
        package="yolo_cone_detection",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("perception")),
        parameters=[
            params,
            {
                "use_sim_time": False,
                "image_topic": image_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "device": "0",
                "publish_annotated": LaunchConfiguration("publish_annotated"),
                "annotated_topic": LaunchConfiguration("annotated_topic"),
            },
        ],
    )

    cone_mapper_node = Node(
        package="cone_mapping",
        executable="cone_mapper_node",
        name="cone_mapper_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("perception")),
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
        DeclareLaunchArgument("camera_model", default_value="zedm",
                              description="ZED model (zedm for ZED-M, zed2i, ...)"),
        DeclareLaunchArgument("image_topic", default_value="/zed2i/zed_node/rgb/color/rect/image",
                              description="wrapper v5 RGB topic"),
        DeclareLaunchArgument("depth_topic", default_value="/zed2i/zed_node/depth/depth_registered"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/zed2i/zed_node/rgb/color/rect/image/camera_info"),
        DeclareLaunchArgument("track_frame", default_value="track",
                              description="Planner/detector world frame"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Vehicle body frame"),
        DeclareLaunchArgument("publish_track_tf", default_value="true",
                              description="Publish static track->base_link (false when "
                              "localization already owns this transform)"),
        DeclareLaunchArgument("publish_camera_tf", default_value="true",
                              description="Publish the static camera mount (false when "
                              "the vehicle URDF already owns base_link->camera_link)"),
        DeclareLaunchArgument("cam_x", default_value="-0.370",
                              description="Camera mount x in base_link [m]"),
        DeclareLaunchArgument("cam_y", default_value="0.0",
                              description="Camera mount y in base_link [m]"),
        DeclareLaunchArgument("cam_z", default_value="0.697",
                              description="Camera mount z in base_link [m]"),
        DeclareLaunchArgument("cam_roll", default_value="0.0",
                              description="Camera mount roll [rad]"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0",
                              description="Camera mount pitch [rad], negative = tilted down"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0",
                              description="Camera mount yaw [rad]"),
        DeclareLaunchArgument("perception", default_value="true",
                              description="Start YOLO detector and cone mapper "
                              "(false = camera + TF only)"),
        DeclareLaunchArgument("publish_annotated", default_value="false",
                              description="Publish YOLO boxes on "
                              "annotated_topic for debugging"),
        DeclareLaunchArgument("annotated_topic",
                              default_value="/yolo/annotated_image"),
        zed_node,
        track_tf,
        mount_tf,
        yolo_node,
        cone_mapper_node,
    ])
