#!/usr/bin/env python3
"""One-command YOLO cold-start warm-up for the skidpad mode (simulation only).

The skidpad fit gate (40 boundary cones, 16 per circle) cannot be met from
the EUFS start line: the simulated camera (10 m, 110 deg) sees 13-26
blue/yellow boundary cones even while driving the entry straight, because
most of both circles stays beside/behind the forward FOV. The production
skidpad planner therefore waits forever and the car never moves.

This script performs the acceptance round-2 warm-up automatically against a
RUNNING simulation (yolo_autonomous_loop with track:=skidpad planner:=skidpad):

  0. stop any running skidpad planner (exactly one path publisher at a time);
  1. spawn a temporary skidpad planner fed with the GT cone map; it drives
     the figure-eight while the live YOLO detector / cone_mapper accumulate
     /mapping/cone_map;
  2. once the GT lap reaches LEFT_1 the map holds ~60 boundary cones: stop
     the temporary planner;
  3. reset the car to the start line (/system/reset_car_pos);
  4. spawn the production planner on /mapping/cone_map; it fits the warm map
     at the start pose and drives the real figure-eight.

The production planner consumes /mapping/cone_map only; the GT map is used
exclusively by the temporary warm-up planner (launch-arg style remap, same
as acceptance round 1/2).

Usage (second terminal, while the sim runs):
  python3 src/planners/scripts/skidpad_yolo_warmup.py
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from driverless_msgs.msg import ConeDetectionStamped
from std_msgs.msg import String

# The script's stdout is line-buffered even under nohup, so its phase
# messages interleave correctly with the child planners' logs.
sys.stdout.reconfigure(line_buffering=True)


def _state_qos() -> QoSProfile:
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class WarmupClient(Node):
    def __init__(self, state_topic: str):
        super().__init__("skidpad_yolo_warmup")
        self.state: dict | None = None
        self.boundary_count: int | None = None
        self.create_subscription(String, state_topic, self._cb, _state_qos())
        self.create_subscription(
            ConeDetectionStamped, "/mapping/cone_map", self._map_cb, _state_qos()
        )
        self.reset_cli = self.create_client(Trigger, "/system/reset_car_pos")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _cb(self, msg: String) -> None:
        try:
            self.state = json.loads(msg.data)
        except (TypeError, ValueError):
            pass

    def _map_cb(self, msg: ConeDetectionStamped) -> None:
        # Boundary-cone count of the live YOLO map. A count far above the
        # track's 60 means the map is fragmented (e.g. repeated warm laps in
        # one session) and the fit's 20% outlier cap will likely reject it.
        self.boundary_count = sum(1 for c in msg.cones if int(c.color) in (0, 4))

    def spin_states(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def car_pose(self):
        """(x, y, yaw) in the track frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform("track", "base_link", rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), float(yaw)

    def reset_and_verify(self, attempts: int = 5) -> bool:
        """Reset the car and wait until TF confirms the start pose.

        The EUFS plugin never sets response.success, and a planner that is
        still shutting down can drag the car after the teleport, so the
        result is verified from TF before continuing.
        """
        if not self.reset_cli.wait_for_service(timeout_sec=5.0):
            print("WARN: /system/reset_car_pos not available (real car?); skipping reset")
            return True
        for k in range(attempts):
            future = self.reset_cli.call_async(Trigger.Request())
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            deadline = time.monotonic() + 3.0
            pose = None
            while time.monotonic() < deadline:
                self.spin_states(0.2)
                pose = self.car_pose()
                if pose is not None:
                    break
            if pose is not None and abs(pose[0]) < 0.3 and abs(pose[1]) < 0.3 and abs(pose[2]) < 0.1:
                print(f"car at start pose: ({pose[0]:.2f}, {pose[1]:.2f}, "
                      f"{math.degrees(pose[2]):.1f} deg)")
                return True
            print(f"WARN: reset attempt {k + 1}: car at {pose}, retrying")
        return False


PLANNER_EXEC = "planners/lib/planners/skidpad_planner_node"


def spawn_planner(params_file: str, overrides: list[str]) -> subprocess.Popen:
    cmd = [
        "ros2", "run", "planners", "skidpad_planner_node", "--ros-args",
        "--params-file", params_file,
    ]
    for o in overrides:
        cmd += ["-p", o]
    return subprocess.Popen(cmd)


def stop_planner(proc: subprocess.Popen) -> None:
    """Stop the spawned planner AND its child, then wait until none remain.

    ``ros2 run`` exits as soon as its wrapper dies, but the planner child can
    briefly outlive it; a planner alive during the car reset would see the
    teleport as a deviation and FAULT, or drag the car with stale windows.
    """
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        alive = subprocess.run(
            ["pgrep", "-f", PLANNER_EXEC], capture_output=True
        ).returncode
        if alive != 0:
            return
        subprocess.run(["pkill", "-f", PLANNER_EXEC], capture_output=True)
        time.sleep(0.5)
    subprocess.run(["pkill", "-9", "-f", PLANNER_EXEC], capture_output=True)
    time.sleep(1.0)


def kill_all_planners() -> None:
    """Stop any skidpad planner (launch-spawned or standalone)."""
    subprocess.run(["pkill", "-f", PLANNER_EXEC], capture_output=True)
    time.sleep(2.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt-topic", default="/ground_truth/global_map")
    ap.add_argument("--warmup-state-topic", default="/warmup/skidpad_state")
    ap.add_argument("--production-state-topic", default="/planning/skidpad_state")
    ap.add_argument("--target-lap", type=int, default=3,
                    help="warm-up lap (segment) at which the GT planner is stopped")
    ap.add_argument("--state-timeout", type=float, default=240.0,
                    help="max seconds for the warm-up lap phase")
    args = ap.parse_args()

    params_file = str(
        get_package_share_path("planners") / "config" / "skidpad.yaml"
    )

    rclpy.init()
    node = WarmupClient(args.production_state_topic)
    # DDS discovery can take a few seconds on a freshly launched sim. Only
    # the SIM must be alive here (TF); the production planner is optional
    # because phase 0 stops it anyway.
    deadline = time.monotonic() + 12.0
    pose = None
    while time.monotonic() < deadline:
        node.spin_states(0.5)
        pose = node.car_pose()
        if pose is not None:
            break
    if pose is None:
        print("ERROR: no track->base_link TF; start the sim first "
              "(yolo_autonomous_loop, track:=skidpad)")
        return 2
    node.spin_states(2.0)
    if node.boundary_count is not None and node.boundary_count > 120:
        print(f"WARN: the live YOLO map holds {node.boundary_count} boundary "
              "cones (the track has 60). The map is fragmented, most likely "
              "from repeated warm laps in this session; the fit's 20% "
              "outlier cap may never pass. Restarting the simulation gives "
              "a fresh cone_mapper.")

    print("[0/4] stopping any running skidpad planner ...")
    kill_all_planners()
    # Rebind the subscription to the warm-up planner's state topic.
    node.create_subscription(String, args.warmup_state_topic, node._cb, _state_qos())

    print(f"[1/4] spawning warm-up planner on {args.gt_topic} ...")
    warm = spawn_planner(params_file, [
        f"cone_map_topic:={args.gt_topic}",
        f"state_topic:={args.warmup_state_topic}",
        "full_path_topic:=/warmup/skidpad_full_path",
        "marker_topic:=/warmup/skidpad_markers",
        "use_sim_time:=true",
    ])

    def bail(code: int, msg: str) -> int:
        # Error paths leave no planner behind: a half-alive planner would
        # either double-drive or keep the map in a bad state.
        print(f"ERROR: {msg}")
        try:
            stop_planner(warm)
        except Exception:  # noqa: BLE001
            pass
        kill_all_planners()
        return code

    deadline = time.monotonic() + args.state_timeout
    while time.monotonic() < deadline:
        node.spin_states(0.5)
        w = node.state or {}
        if w.get("state") == "FAULT":
            return bail(3, "warm-up lap ended in FAULT; the tracker lost the "
                           "GT path. Check the sim state and retry.")
        if int(w.get("lap", 0)) >= args.target_lap or w.get("segment") in (
            "LEFT_1", "LEFT_2", "EXIT"
        ):
            print(f"[2/4] warm map ready ({w.get('segment')}, lap "
                  f"{w.get('lap')}); stopping warm-up planner ...")
            break
        if warm.poll() is not None:
            return bail(3, "warm-up planner exited unexpectedly")
    else:
        return bail(3, f"warm-up timeout; GT lap did not reach lap {args.target_lap}")
    stop_planner(warm)

    print("[3/4] resetting the car to the start line ...")
    if not node.reset_and_verify():
        return bail(3, "car did not return to the start pose")
    time.sleep(1.0)

    # Back to the production state topic for the final phase.
    node.state = None
    node.create_subscription(
        String, args.production_state_topic, node._cb, _state_qos()
    )
    print("[4/4] spawning production planner on the live YOLO map ...")
    prod = spawn_planner(params_file, ["use_sim_time:=true"])
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline:
        node.spin_states(0.5)
        s = node.state or {}
        if s.get("fit_ok") and s.get("state") == "DRIVING":
            break
        if s.get("state") == "FAULT":
            return bail(3, "production planner drove into FAULT")
        if prod.poll() is not None:
            return bail(3, "production planner exited unexpectedly")
    else:
        return bail(3, "production planner did not start driving on the warm map")

    # SUCCESS: leave the production planner running (it owns the run now);
    # never clean up on this path.
    print(
        "OK: the production planner is driving the figure-eight from the "
        f"live YOLO map (s_prog={s.get('s_prog')}, n={s.get('boundary_cones')}). "
        f"Monitor {args.production_state_topic} for lap progress."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
