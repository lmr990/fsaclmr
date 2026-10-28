"""Cone mapper ROS2 node.

Accumulates cone detections from an absolute-pose source (any detector
publishing `driverless_msgs/ConeDetectionStamped` in a map frame — the
YOLO node by default) into a merged cone map: observation-count weighted
positions, colour majority voting, min-observation gating and stale
eviction. Publishes the confirmed map for the planner.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance
from driverless_common.common import QOS_LATEST

from visualization_msgs.msg import Marker, MarkerArray

from cone_mapping.cone_mapper import (
    CONE_BLUE,
    CONE_ORANGE_BIG,
    CONE_ORANGE_SMALL,
    CONE_UNKNOWN,
    CONE_YELLOW,
    MapCone,
    covariance_for,
    evict_stale,
    merge_detection,
    new_cone,
    publishable,
)

_MARKER_COLORS = {
    CONE_BLUE: (0.12, 0.45, 0.95),
    CONE_YELLOW: (0.98, 0.85, 0.09),
    CONE_ORANGE_BIG: (0.98, 0.45, 0.05),
    CONE_ORANGE_SMALL: (0.98, 0.45, 0.05),
    CONE_UNKNOWN: (0.85, 0.85, 0.85),
}


class ConeMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("cone_mapper_node")

        self.declare_parameter("input_topic", "slam/cone_detection")
        self.declare_parameter("output_topic", "/mapping/cone_map")
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("merge_radius", 0.5)         # [m]
        self.declare_parameter("min_observations", 3)       # gate for the planner
        self.declare_parameter("max_age_sec", 120.0)        # eviction age
        self.declare_parameter("max_cones", 0)              # 0 = unlimited
        self.declare_parameter("publish_rate", 10.0)        # [Hz]
        # If the detector stops publishing entirely (node crash, camera
        # stall), stop trusting the last map instead of republishing it
        # forever. Wall clock, so a paused sim cannot mask the loss.
        self.declare_parameter("input_timeout_sec", 2.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.merge_radius = float(self.get_parameter("merge_radius").value)
        self.min_observations = int(self.get_parameter("min_observations").value)
        self.max_age = float(self.get_parameter("max_age_sec").value)
        self.max_cones = int(self.get_parameter("max_cones").value)
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)

        self.map_cones: list[MapCone] = []
        self.last_header = None
        self.detections_received = 0
        self.last_detection_mono = None
        self.input_lost = False

        self.create_subscription(
            ConeDetectionStamped, input_topic, self.detection_callback, QOS_LATEST
        )
        self.map_pub = self.create_publisher(ConeDetectionStamped, output_topic, QOS_LATEST)
        self.marker_pub = None
        if self.get_parameter("publish_markers").value:
            self.marker_pub = self.create_publisher(
                MarkerArray, "/mapping/cone_map_viz", 1
            )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self.publish_map)

        self.get_logger().info(
            f"---Cone mapper initialised: {input_topic} -> {output_topic} "
            f"(merge_radius={self.merge_radius}m, min_obs={self.min_observations})---"
        )

    # ------------------------------------------------------------------ #
    def detection_callback(self, msg: ConeDetectionStamped) -> None:
        self.last_header = msg.header
        self.last_detection_mono = time.monotonic()
        self.input_lost = False
        # node clock (respects use_sim_time) so ages survive sim replay
        now = self.get_clock().now().nanoseconds / 1e9

        for cone in msg.cones:
            self.detections_received += 1
            merged, _ = merge_detection(
                self.map_cones,
                cone.location.x,
                cone.location.y,
                cone.color,
                now,
                self.merge_radius,
                cone.sim_cone_index,
            )
            if not merged:
                self.map_cones.append(
                    new_cone(
                        cone.location.x, cone.location.y, cone.color, now, cone.sim_cone_index
                    )
                )

        evicted = evict_stale(self.map_cones, now, self.max_age)
        if evicted:
            self.get_logger().debug(f"evicted {evicted} stale cones")

    # ------------------------------------------------------------------ #
    def publish_map(self) -> None:
        if self.last_header is None or self.last_detection_mono is None:
            return

        # Detector silence: drop the map so downstream planners fall back to
        # their freshness checks instead of driving on a stale map.
        silent_for = time.monotonic() - self.last_detection_mono
        if silent_for > self.input_timeout and not self.input_lost:
            self.input_lost = True
            self.map_cones.clear()
            self.get_logger().warn(
                f"No detections for {silent_for:.1f}s (> {self.input_timeout:.1f}s); "
                "clearing cone map"
            )

        confirmed = publishable(self.map_cones, self.min_observations, self.max_cones)

        map_msg = ConeDetectionStamped()
        map_msg.header = self.last_header
        for c in confirmed:
            cone = Cone()
            cone.location.x = c.x
            cone.location.y = c.y
            cone.location.z = 0.0
            cone.color = c.color
            cone.sim_cone_index = c.sim_cone_index
            map_msg.cones.append(cone)

            cone_cov = ConeWithCovariance()
            cone_cov.cone = cone
            cone_cov.covariance = covariance_for(c.obs_count)
            map_msg.cones_with_cov.append(cone_cov)

        self.map_pub.publish(map_msg)

        if self.marker_pub is not None:
            self.publish_markers(confirmed)

        self.get_logger().info(
            f"map: {len(confirmed)}/{len(self.map_cones)} cones "
            f"({self.detections_received} detections seen)",
            throttle_duration_sec=5.0,
        )

    def publish_markers(self, cones: list[MapCone]) -> None:
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, c in enumerate(cones):
            m = Marker()
            m.header = self.last_header
            m.ns = "cone_map"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c.x
            m.pose.position.y = c.y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            color = _MARKER_COLORS.get(c.color, _MARKER_COLORS[CONE_UNKNOWN])
            m.color.r, m.color.g, m.color.b = color
            m.color.a = min(1.0, 0.3 + 0.1 * c.obs_count)
            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
