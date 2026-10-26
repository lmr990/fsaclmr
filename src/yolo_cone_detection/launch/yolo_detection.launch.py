"""Launch the YOLO cone detection node."""

import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_path("yolo_cone_detection"), "config", "yolo_detector.yaml"
    )

    tcone_yolo_path = LaunchConfiguration("tcone_yolo_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    yolo_node = Node(
        package="yolo_cone_detection",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "tcone_yolo_path": tcone_yolo_path,
            },
        ],
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "tcone_yolo_path",
                default_value="/home/user/qutms_ws/src/TCone-YOLO-main",
                description="Absolute path to the TCone-YOLO repository root",
            ),
            yolo_node,
        ]
    )
