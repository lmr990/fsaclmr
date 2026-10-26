"""Launch the static-camera path demo:

    webcam_path_node (detect + monocular distance + overlay display)
      -> slam/cone_detection
    cone_mapper_node  -> /mapping/cone_map
    ft_planner_node   -> /planning/midline_path (needs a car pose TF)
    static TF  track -> base_link (car at origin, identity)
    mock /vehicle/velocity = 0 (planner speed seeding)

No simulation required. Use source:=file to replay captured images.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("webcam_path_demo")
    params = os.path.join(pkg, "config", "webcam_path_demo.yaml")
    planner_params = os.path.join(
        get_package_share_directory("planners"), "config", "ft_planner.yaml")

    source = LaunchConfiguration("source")
    display = LaunchConfiguration("display")
    distance_method = LaunchConfiguration("distance_method")
    intrinsics_file = LaunchConfiguration("intrinsics_file")
    save_debug_png = LaunchConfiguration("save_debug_png")
    image_files = LaunchConfiguration("image_files")

    webcam_node = Node(
        package="webcam_path_demo",
        executable="webcam_path_node",
        name="webcam_path_node",
        output="screen",
        parameters=[
            params,
            {
                "source": source,
                "display": display,
                "distance_method": distance_method,
                "intrinsics_file": intrinsics_file,
                "save_debug_png": save_debug_png,
                "image_files": image_files,
                "use_sim_time": False,
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
                "min_observations": 2,
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
        DeclareLaunchArgument("source", default_value="camera",
                              description="camera | file (replay captured images)"),
        DeclareLaunchArgument("display", default_value="true",
                              description="show the OpenCV overlay window"),
        DeclareLaunchArgument("distance_method", default_value="size",
                              description="size | ground"),
        DeclareLaunchArgument("intrinsics_file", default_value="",
                              description="yaml with K from webcam_calibrate"),
        DeclareLaunchArgument("save_debug_png", default_value="",
                              description="save the latest overlay to this path"),
        DeclareLaunchArgument("image_files", default_value="",
                              description="glob of images for source:=file"),
        webcam_node,
        cone_mapper_node,
        ft_planner_node,
        static_tf,
        mock_velocity,
    ])
