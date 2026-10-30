"""Skidpad mission handler.

Same lifecycle as the trackdrive handler (AV/ROS state, debug launch,
ShutdownNode safety), but it does not send a Nav2 goal: the YOLO pipeline's
path tracker consumes /planning/path_with_speeds directly. Lap completion is
read from planning/skidpad_state and FINISHED triggers the normal shutdown.
"""

from __future__ import annotations

import json
import time
from subprocess import Popen

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from driverless_msgs.msg import AVStateStamped, ROSStateStamped, Shutdown
from std_msgs.msg import String, UInt8

from vehicle_bringup.shutdown_node_class import ShutdownNode


class SkidpadHandler(ShutdownNode):
    mission_started = False
    sbg_operational = False
    debug = False
    released = False
    finished = False
    last_lap = 0

    def __init__(self):
        super().__init__("skidpad_handler_node")

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("debug", False)
        self.declare_parameter("mission_launch", "skidpad.launch.py")
        self.declare_parameter("state_topic", "planning/skidpad_state")
        state_topic = str(self.get_parameter("state_topic").value)

        self.create_subscription(
            AVStateStamped,
            "system/av_state",
            self.av_state_callback,
            1,
            callback_group=self.sub_cb_group,
        )
        self.create_subscription(
            ROSStateStamped, "system/ros_state", self.ros_state_callback, 1
        )
        self.create_subscription(String, state_topic, self.state_callback, 1)

        self.shutdown_pub = self.create_publisher(Shutdown, "system/shutdown", 1)
        self.lap_trig_pub = self.create_publisher(UInt8, "system/laps_completed", 1)

        if bool(self.get_parameter("debug").value):
            self.get_logger().warn("---DEBUG MODE ENABLED---")
            self.mission_started = True
            self.released = True
            self.sbg_operational = True
            self._start_mission()

        self.get_logger().info("---Skidpad handler node initialised---")

    # ------------------------------------------------------------------ #
    def _start_mission(self) -> None:
        command = [
            "stdbuf",
            "-o",
            "L",
            "ros2",
            "launch",
            "vehicle_bringup",
            str(self.get_parameter("mission_launch").value),
        ]
        self.get_logger().info(f"Command: {' '.join(command)}")
        self.mission_process = Popen(command)
        self.get_logger().info("Skidpad mission started")

    # ------------------------------------------------------------------ #
    def av_state_callback(self, msg: AVStateStamped):
        super().av_state_callback(msg)
        if (
            (msg.state == AVStateStamped.START_MISSION or msg.state == AVStateStamped.DRIVING)
            and not self.mission_started
            and self.sbg_operational
        ):
            self.mission_started = True
            self._start_mission()

        if msg.state == AVStateStamped.DRIVING and not self.released:
            time.sleep(3)
            self.released = True
            self.get_logger().info("RELEASED BRAKES")

    def ros_state_callback(self, msg: ROSStateStamped):
        if msg.sbg_operational:
            self.sbg_operational = True

    # ------------------------------------------------------------------ #
    def state_callback(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        phase = str(state.get("state", ""))
        lap = int(state.get("lap", 0))

        # The state lap is 1..4 for the four FSAC circles; EXIT is 5.
        if 1 <= lap <= 4 and lap > self.last_lap:
            self.last_lap = lap
            self.lap_trig_pub.publish(UInt8(data=lap))
            self.get_logger().info(
                f"Skidpad lap {lap} ({state.get('segment', '?')}); "
                f"s_prog={state.get('s_prog', '?')}"
            )

        if phase == "FINISHED" and not self.finished:
            self.finished = True
            self.get_logger().info("Skidpad mission complete")
            shutdown = Shutdown()
            shutdown.finished_engage_ebs = True
            self.shutdown_pub.publish(shutdown)


def main(args=None):
    rclpy.init(args=args)
    node = SkidpadHandler()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
