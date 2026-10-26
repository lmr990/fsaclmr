import datetime
import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    planner = LaunchConfiguration("planner")
    use_urinay = PythonExpression(["'", planner, "' == 'urinay'"])
    use_ft = PythonExpression(["'", planner, "' == 'ft'"])
    # nav2 stack
    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path("nav_bringup"), "launch", "nav_stack_bringup.launch.py")
        ),
        launch_arguments=[
            ("use_sim_time", "True"),
            # ("use_sim_time", "False"),
        ],
    )
    # mapping/planning
    grid_to_cone_node = Node(
        package="slam_gridmap",
        executable="gridmap_to_cone_detection_node",
        output="both",
    )
    planner_node = Node(
        package="planners",
        executable="ft_planner_node",
        condition=IfCondition(use_ft),
        parameters=[
            get_package_share_path("planners") / "config" / "ft_planner.yaml",
        ],
        output="both",
    )
    # urinay chain: cone_id_mapper -> urinay_exec -> urinay_path_bridge
    cone_id_mapper_node = Node(
        package="planners",
        executable="cone_id_mapper_node",
        condition=IfCondition(use_urinay),
        parameters=[
            get_package_share_path("planners") / "config" / "urinay.yaml",
        ],
        output="both",
    )
    urinay_node = Node(
        package="urinay",
        executable="urinay_exec",
        condition=IfCondition(use_urinay),
        parameters=[
            get_package_share_path("urinay") / "config" / "urinay.yaml",
        ],
        output="both",
    )
    urinay_path_bridge_node = Node(
        package="planners",
        executable="urinay_path_bridge_node",
        condition=IfCondition(use_urinay),
        parameters=[
            get_package_share_path("planners") / "config" / "urinay.yaml",
        ],
        output="both",
    )
    # control translation
    ackermann_control_node = Node(
        package="controllers",
        executable="vel_to_ackermann_node",
        parameters=[
            get_package_share_path("controllers") / "config" / "control.yaml",
        ],
        output="both",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            DeclareLaunchArgument(
                "planner",
                default_value="urinay",
                description="Path planner: urinay (color-blind Delaunay) or ft (FaSTTUBe)",
            ),
            nav_stack_launch,
            grid_to_cone_node,
            planner_node,
            cone_id_mapper_node,
            urinay_node,
            urinay_path_bridge_node,
            ackermann_control_node,
        ]
    )
