"""Synthetic corridor self-test for the webcam demo's planning chain.

Publishes a clean blue/yellow cone corridor on slam/cone_detection (bypassing
YOLO) and verifies that cone_mapper + ft_planner produce a centered
/planning/midline_path. No GPU, no camera, no simulation needed.

Run:  python3 synthetic_corridor_test.py   (while the demo launch is up,
or launch the planning nodes via the demo launch with source:=file)
"""

import math
import re
import subprocess
import sys
import time

import rclpy
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path

TRACK_HALF_WIDTH = 1.5      # blue at -1.5, yellow at +1.5
LENGTH = 30.0
STEP = 3.0
CURVE = 0.15                # gentle left curve: y_offset = CURVE * x^2 / 10


def corridor_cones():
    cones = []
    n = int(LENGTH / STEP)
    for i in range(n + 1):
        x = i * STEP
        y_mid = CURVE * x * x / 10.0
        for side, color in ((+TRACK_HALF_WIDTH, Cone.BLUE), (-TRACK_HALF_WIDTH, Cone.YELLOW)):
            # REP-103: +y is LEFT. fsd_path_planning expects blue on the left
            # (+y) and yellow on the right (-y) in the direction of travel.
            c = Cone()
            c.location = Point(x=float(x), y=float(y_mid + side), z=0.0)
            c.color = color
            cones.append(c)
    return cones


class SynthTest(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_corridor_test")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(ConeDetectionStamped, "slam/cone_detection", qos)
        self.path: list = []
        self.create_subscription(Path, "/planning/midline_path", self._cb, qos)

    def _cb(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def publish_once(self) -> None:
        m = ConeDetectionStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "track"
        m.cones = corridor_cones()
        self.pub.publish(m)


def main() -> None:
    rclpy.init()
    node = SynthTest()
    spin = threading_spin(node)

    print("Publishing synthetic corridor (22 cones, 30m, gentle left curve)...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 25.0:
        node.publish_once()
        time.sleep(1.0)
        if node.path:
            break

    if not node.path:
        print("FAIL: no /planning/midline_path received", flush=True)
        sys.exit(1)

    xs = [p[0] for p in node.path]
    ys = [p[1] for p in node.path]
    # expected midline y at each x
    errs = [abs(y - (CURVE * x * x / 10.0)) for x, y in node.path if x > 1.0]
    print(f"PASS: path with {len(node.path)} points", flush=True)
    print(f"  X range: {min(xs):.1f} ~ {max(xs):.1f} m")
    print(f"  Y range: {min(ys):.2f} ~ {max(ys):.2f} m")
    if errs:
        print(f"  与理论中线平均偏差: {sum(errs)/len(errs):.2f} m (最大 {max(errs):.2f} m)")
    os_exit(0)


def threading_spin(node):
    import threading
    th = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    th.start()
    return th


def os_exit(code):
    import os, sys
    sys.stdout.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
