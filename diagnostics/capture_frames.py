"""Capture real camera frames from the EUFS simulation for offline benchmarks.

Subscribes to the ZED RGB/depth/camera_info topics, saves a few frames to
diagnostics/frames/ and exits. Run while the simulation is up.
"""

import os
import sys

import numpy as np
import rclpy
import cv2
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frames")
N_FRAMES = 5


class FrameCapture(Node):
    def __init__(self) -> None:
        super().__init__("frame_capture")
        self.bridge = CvBridge()
        self.rgb: list = []
        self.depth: list = []
        self.info: list = []
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "/zed2i/zed_node/rgb/image_rect_color", self._rgb, qos)
        self.create_subscription(Image, "/zed2i/zed_node/depth/depth_registered", self._depth, qos)
        self.create_subscription(CameraInfo, "/zed2i/zed_node/rgb/camera_info", self._info, qos)

    def _rgb(self, msg: Image) -> None:
        if len(self.rgb) < N_FRAMES:
            self.rgb.append(self.bridge.imgmsg_to_cv2(msg, "bgr8"))
            self._maybe_save()

    def _depth(self, msg: Image) -> None:
        if len(self.depth) < N_FRAMES:
            self.depth.append(self.bridge.imgmsg_to_cv2(msg, "32FC1"))
            self._maybe_save()

    def _info(self, msg: CameraInfo) -> None:
        if len(self.info) < 1:
            self.info.append(msg.k)
            self._maybe_save()

    def _maybe_save(self) -> None:
        if len(self.rgb) >= N_FRAMES and len(self.depth) >= N_FRAMES and len(self.info) >= 1:
            os.makedirs(OUT, exist_ok=True)
            for i, im in enumerate(self.rgb):
                cv2.imwrite(os.path.join(OUT, f"rgb_{i}.png"), im)
            for i, d in enumerate(self.depth):
                np.save(os.path.join(OUT, f"depth_{i}.npy"), d)
            with open(os.path.join(OUT, "camera_info.yaml"), "w") as f:
                yaml.dump(np.asarray(self.info[0]).tolist(), f)
            self.get_logger().info(f"Saved {N_FRAMES} frames to {OUT}")
            raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = FrameCapture()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
