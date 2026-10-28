"""Launch the full YOLO-based autonomous pipeline (upgraded stack).

Includes:
  - yolo_detector_node  : camera -> YOLO -> slam/cone_detection (track frame)
  - cone_mapper_node    : slam/cone_detection -> /mapping/cone_map (weighted merge)
  - planner (choice, `planner:=urinay|ft`, default urinay):
      urinay chain: cone_id_mapper_node + urinay_exec + urinay_path_bridge_node
                    (color-blind centerline from Delaunay triangulation)
      ft:           ft_planner_node (FaSTTUBe fsd_path_planning)
    both publish /planning/path_with_speeds
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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    tcone_yolo_path = LaunchConfiguration("tcone_yolo_path")
    planner = LaunchConfiguration("planner")
    shadow_planner = LaunchConfiguration("shadow_planner")
    use_urinay = PythonExpression(["'", planner, "' == 'urinay'"])
    use_ft = PythonExpression(["'", planner, "' == 'ft'"])
    # Shadow planner: the second chain runs with its output remapped to
    # /shadow/<name>/... and an arbiter node logs how it would have driven.
    # It never publishes to the control topics. Requires planner != shadow.
    shadow_ft = PythonExpression(["'", shadow_planner, "' == 'ft' and '", planner, "' != 'ft'"])
    shadow_urinay = PythonExpression(
        ["'", shadow_planner, "' == 'urinay' and '", planner, "' != 'urinay'"]
    )

    # Track-dependent tuning: every track has its own width, so the urinay
    # edge filter and the bridge corridor guard are adjustable per launch.
    max_track_width = ParameterValue(LaunchConfiguration("max_track_width"), value_type=float)
    corridor_max_dist = ParameterValue(LaunchConfiguration("corridor_max_dist"), value_type=float)

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
    # selected with the `planner` launch argument: urinay (default) or ft
    planner_node = Node(
        package="planners",
        executable="ft_planner_node",
        name="ft_planner_node",
        output="screen",
        condition=IfCondition(use_ft),
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "ft_planner.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---- Urinay planner chain (color-blind centerline + track limits) ----
    # cone_id_mapper: /mapping/cone_map -> stable-id /urinay/cone_map
    # urinay_exec:    Delaunay + tree search -> /AS/P/pathlimits/{partial,full}
    # path bridge:    PathLimits -> /planning/{midline_path,path_with_speeds}
    cone_id_mapper_node = Node(
        package="planners",
        executable="cone_id_mapper_node",
        name="cone_id_mapper_node",
        output="screen",
        condition=IfCondition(use_urinay),
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "urinay.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )
    urinay_node = Node(
        package="urinay",
        executable="urinay_exec",
        name="urinay",
        output="screen",
        condition=IfCondition(use_urinay),
        parameters=[
            os.path.join(get_package_share_path("urinay"), "config", "urinay.yaml"),
            {
                "use_sim_time": use_sim_time,
                "max_track_width": max_track_width,
            },
        ],
    )
    urinay_path_bridge_node = Node(
        package="planners",
        executable="urinay_path_bridge_node",
        name="urinay_path_bridge_node",
        output="screen",
        condition=IfCondition(use_urinay),
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "urinay.yaml"),
            {
                "use_sim_time": use_sim_time,
                "corridor.max_dist_to_cone": corridor_max_dist,
            },
        ],
    )

    # ---- Shadow planner (never drives the car; observed by the arbiter) ----
    ft_shadow_node = Node(
        package="planners",
        executable="ft_planner_node",
        name="ft_planner_shadow",
        output="screen",
        condition=IfCondition(shadow_ft),
        remappings=[
            ("/planning/midline_path", "/shadow/ft/midline_path"),
            ("/planning/path_with_speeds", "/shadow/ft/path_with_speeds"),
            ("/planning/blue_bounds", "/shadow/ft/blue_bounds"),
            ("/planning/yellow_bounds", "/shadow/ft/yellow_bounds"),
        ],
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "ft_planner.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )
    # Shadow urinay chain (only launched when ft is active, so the shared
    # cone_id_mapper / pathlimits topics have no second active publisher).
    cone_id_mapper_shadow_node = Node(
        package="planners",
        executable="cone_id_mapper_node",
        name="cone_id_mapper_shadow",
        output="screen",
        condition=IfCondition(shadow_urinay),
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "urinay.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )
    urinay_shadow_node = Node(
        package="urinay",
        executable="urinay_exec",
        name="urinay_shadow",
        output="screen",
        condition=IfCondition(shadow_urinay),
        parameters=[
            os.path.join(get_package_share_path("urinay"), "config", "urinay.yaml"),
            {
                "use_sim_time": use_sim_time,
                "max_track_width": max_track_width,
            },
        ],
    )
    urinay_bridge_shadow_node = Node(
        package="planners",
        executable="urinay_path_bridge_node",
        name="urinay_bridge_shadow",
        output="screen",
        condition=IfCondition(shadow_urinay),
        parameters=[
            os.path.join(get_package_share_path("planners"), "config", "urinay.yaml"),
            {
                "use_sim_time": use_sim_time,
                "path_topic": "/shadow/urinay/midline_path",
                "path_with_speeds_topic": "/shadow/urinay/path_with_speeds",
                "corridor.max_dist_to_cone": corridor_max_dist,
            },
        ],
    )
    arbiter_ft_node = Node(
        package="planners",
        executable="planner_arbiter_node",
        name="planner_arbiter",
        output="screen",
        condition=IfCondition(shadow_ft),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "active_name": "urinay",
                "shadow_name": "ft",
                "shadow_topic": "/shadow/ft/midline_path",
                "track_name": LaunchConfiguration("track"),
            }
        ],
    )
    arbiter_urinay_node = Node(
        package="planners",
        executable="planner_arbiter_node",
        name="planner_arbiter",
        output="screen",
        condition=IfCondition(shadow_urinay),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "active_name": "ft",
                "shadow_name": "urinay",
                "shadow_topic": "/shadow/urinay/midline_path",
                "track_name": LaunchConfiguration("track"),
            }
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
                default_value="",
                description="Absolute path to TCone-YOLO repo (empty = auto-detect)",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="urinay",
                description="Path planner: urinay (color-blind Delaunay) or ft (FaSTTUBe)",
            ),
            DeclareLaunchArgument(
                "shadow_planner",
                default_value="",
                description="Second planner to run in shadow mode ('ft'/'urinay', "
                "must differ from planner); logs comparisons, never drives",
            ),
            DeclareLaunchArgument(
                "max_track_width",
                default_value="5.0",
                description="Urinay max edge width [m] — set to the track's real "
                "width + ~1.5 m margin when switching tracks",
            ),
            DeclareLaunchArgument(
                "corridor_max_dist",
                default_value="3.0",
                description="Bridge corridor guard: max path-to-nearest-cone "
                "distance [m] — set to the track's half width + ~1.5 m margin",
            ),
            yolo_node,
            cone_mapper_node,
            planner_node,
            cone_id_mapper_node,
            urinay_node,
            urinay_path_bridge_node,
            ft_shadow_node,
            cone_id_mapper_shadow_node,
            urinay_shadow_node,
            urinay_bridge_shadow_node,
            arbiter_ft_node,
            arbiter_urinay_node,
            path_tracker_node,
            safety_watchdog_node,
        ]
    )
