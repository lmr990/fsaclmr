"""YOLO cone detection ROS2 node.

Subscribes to a stereo camera's RGB image, registered depth image and
camera info, runs TCone-YOLO inference, back-projects detections into 3D
using the depth map, transforms them into the output frame (default
``track``) and publishes a ``driverless_msgs/ConeDetectionStamped`` message
to ``slam/cone_detection`` so the path planner can consume it directly.
"""

from __future__ import annotations

import os
import time
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from driverless_msgs.msg import Cone, ConeDetectionStamped, ConeWithCovariance
from driverless_common.common import QOS_LATEST, FPSHandler

from yolo_cone_detection.yolo_inference import YoloInference


# TCone-YOLO class index -> driverless_msgs Cone colour constant.
# TCone-YOLO Cone.yaml: 0: red, 1: blue, 2: yellow
CLASS_TO_COLOR = {
    0: Cone.ORANGE_SMALL,  # red -> orange (small/large indistinguishable from image)
    1: Cone.BLUE,
    2: Cone.YELLOW,
}

# Debug overlay (only drawn when publish_annotated is true).
_CLASS_BGR = {0: (0, 100, 255), 1: (255, 120, 0), 2: (0, 220, 255)}
_CLASS_LABEL = {0: "orange", 1: "blue", 2: "yellow"}


def resolve_tcone_root(configured: str) -> Path:
    """Locate the TCone-YOLO repository without hard-coding one workspace copy.

    Order: explicit parameter -> QUTMS_TCONE_YOLO_PATH -> any
    ``src/TCone-YOLO-main`` above the current working directory or this file
    (covers a workspace snapshot such as ``qutms_ws (1)``) -> ~/qutms_ws/src.
    Workspace-relative candidates come first so a copy of the workspace does
    not silently use another copy's repository.
    """
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    env_path = os.environ.get("QUTMS_TCONE_YOLO_PATH")
    if env_path:
        candidates.append(Path(env_path).expanduser())

    for base in (Path.cwd(), Path(__file__).resolve().parent):
        for ancestor in [base, *base.parents]:
            candidates.append(ancestor / "src" / "TCone-YOLO-main")

    candidates.append(Path.home() / "qutms_ws" / "src" / "TCone-YOLO-main")

    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()
    raise FileNotFoundError(
        "TCone-YOLO repository not found. Set the tcone_yolo_path parameter, "
        "the QUTMS_TCONE_YOLO_PATH environment variable, or place it at "
        "~/qutms_ws/src/TCone-YOLO-main. Tried: "
        + ", ".join(str(c) for c in candidates)
    )


class YoloDetectorNode(Node):
    """ROS2 node performing YOLO-based cone detection with depth back-projection."""

    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        # ----- Parameters -------------------------------------------------
        self.declare_parameter("weights", "")
        self.declare_parameter("tcone_yolo_path", "")
        self.declare_parameter("data_yaml", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 1000)
        self.declare_parameter("device", "")  # '' = auto (cuda if available else cpu)
        self.declare_parameter("half", False)
        self.declare_parameter("output_frame", "track")
        self.declare_parameter("min_range", 0.5)
        # Match the ZED2i depth camera effective operating range used in the
        # simulation and URDF; beyond this, depth becomes invalid/NaN.
        self.declare_parameter("max_range", 15.0)
        # Real-world cone height (m) used to estimate distance from the
        # apparent bounding-box height when the depth sensor returns no value.
        self.declare_parameter("cone_real_height", 0.305)
        self.declare_parameter("image_topic", "/zed2i/zed_node/rgb/image_rect_color")
        self.declare_parameter("depth_topic", "/zed2i/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_topic", "/zed2i/zed_node/rgb/camera_info")
        self.declare_parameter("publish_topic", "slam/cone_detection")
        # Debug image with the raw 2D boxes for GUI/rqt debugging.
        self.declare_parameter("publish_annotated", False)
        self.declare_parameter("annotated_topic", "/yolo/annotated_image")
        # Timer period ceiling for the detection rate [s]. The timer always
        # processes the newest frame and self-paces if inference is slower.
        self.declare_parameter("process_period_sec", 0.1)

        tcone_yolo_path = self.get_parameter("tcone_yolo_path").value
        weights = self.get_parameter("weights").value
        data_yaml = self.get_parameter("data_yaml").value
        device = self.get_parameter("device").value
        half = self.get_parameter("half").value

        self.output_frame = self.get_parameter("output_frame").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.cone_real_height = float(self.get_parameter("cone_real_height").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.publish_topic = self.get_parameter("publish_topic").value

        # ----- Resolve and validate paths ---------------------------------
        try:
            tcone_root = resolve_tcone_root(tcone_yolo_path)
        except FileNotFoundError as exc:
            self.get_logger().fatal(str(exc))
            raise

        weights_path = Path(weights)
        if not weights_path.is_absolute():
            weights_path = tcone_root / weights_path
        if not weights_path.exists():
            self.get_logger().fatal(f"Weights file not found: {weights_path}")
            raise FileNotFoundError(f"Weights file not found: {weights_path}")

        data_yaml_path = Path(data_yaml)
        if not data_yaml_path.is_absolute():
            data_yaml_path = tcone_root / data_yaml_path
        if not data_yaml_path.exists():
            self.get_logger().fatal(f"data yaml not found: {data_yaml_path}")
            raise FileNotFoundError(f"data yaml not found: {data_yaml_path}")

        weights = str(weights_path)
        data_yaml = str(data_yaml_path)
        tcone_yolo_path = str(tcone_root)

        # ----- YOLO model -------------------------------------------------
        self.inference = YoloInference(
            weights=weights,
            data_yaml=data_yaml,
            tcone_root=tcone_yolo_path,
            imgsz=int(self.get_parameter("imgsz").value),
            conf_thres=float(self.get_parameter("conf_thres").value),
            iou_thres=float(self.get_parameter("iou_thres").value),
            max_det=int(self.get_parameter("max_det").value),
            device=device,
            half=half,
        )

        # ----- TF ---------------------------------------------------------
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- CV bridge --------------------------------------------------
        self.cv_bridge = CvBridge()

        # ----- Cached latest messages ------------------------------------
        self._latest_camera_info: Optional[CameraInfo] = None
        # Keep the most recent depth frames with their stamps: the RGB frame
        # driving a detection must be paired with the depth captured closest
        # in time, otherwise every reading is taken from a different viewpoint
        # while the car moves (source of ghost cones in the map).
        self._depth_buffer: deque = deque(maxlen=8)
        # Inference takes ~0.1-0.3 s; this timer period is the ceiling for the
        # detection rate and self-paces when inference is slower.
        self._min_process_period = max(
            0.01, float(self.get_parameter("process_period_sec").value)
        )
        # Only the newest RGB frame is kept; the timer consumes it exactly once.
        self._pending_bgr = None

        # ----- Subscribers / Publisher ------------------------------------
        # Sensor data QoS (BEST_EFFORT) matches typical camera publishers.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, self.image_topic, self._image_cb, sensor_qos)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, QOS_LATEST)

        self.detection_pub = self.create_publisher(ConeDetectionStamped, self.publish_topic, QOS_LATEST)
        self.annotated_pub = None
        if bool(self.get_parameter("publish_annotated").value):
            self.annotated_pub = self.create_publisher(
                Image, self.get_parameter("annotated_topic").value, QOS_LATEST
            )

        # Processing timer: always works on the newest stored frame.
        self.create_timer(self._min_process_period, self._process_timer)

        self.fps = FPSHandler()

        # Warm up the model after a short delay so the node is ready to
        # receive images (model loading is heavy on first call).
        self.get_logger().info("Loading YOLO model (this may take a few seconds)...")
        start = time.perf_counter()
        try:
            self.inference.warmup()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(f"Failed to load YOLO model: {exc}")
            raise
        elapsed = time.perf_counter() - start
        self.get_logger().info(
            f"YOLO model loaded in {elapsed:.2f}s. "
            f"Classes: {self.inference.names}"
        )
        self.get_logger().info("---YOLO detector node initialised---")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _depth_cb(self, msg: Image) -> None:
        try:
            # Gazebo depth camera outputs 32FC1 (float32 metres).
            depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert depth image: {exc}", throttle_duration_sec=2.0)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._depth_buffer.append((stamp, depth, msg.header.frame_id))

    def _image_cb(self, msg: Image) -> None:
        """Store the newest RGB frame only; processing happens on a timer."""
        self._pending_bgr = msg

    def _process_timer(self) -> None:
        """Run one inference on the newest stored frame (timer-paced)."""
        if self._latest_camera_info is None or not self._depth_buffer:
            return
        if self._pending_bgr is None:
            return

        # Consume the frame exactly once. Without this, a stalled camera would
        # be re-inferred and re-published with its old stamp forever.
        msg = self._pending_bgr
        self._pending_bgr = None

        try:
            bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert RGB image: {exc}", throttle_duration_sec=2.0)
            return

        color_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        depth_stamp, depth, depth_frame = min(
            self._depth_buffer, key=lambda e: abs(e[0] - color_stamp)
        )
        camera_frame = msg.header.frame_id or depth_frame
        if abs(depth_stamp - color_stamp) > 0.2:
            self.get_logger().warn(
                f"Depth/RGB stamp mismatch {abs(depth_stamp - color_stamp):.3f}s; "
                "detections may be misplaced while moving",
                throttle_duration_sec=5.0,
            )
        if not camera_frame:
            self.get_logger().warn("No camera frame id available, skipping frame", throttle_duration_sec=2.0)
            return

        # Run inference
        t0 = time.perf_counter()
        detections = self.inference.detect(bgr)
        infer_ms = (time.perf_counter() - t0) * 1000.0

        cones, cones_cov = self._build_cones(
            detections=detections,
            camera_info=self._latest_camera_info,
            depth=depth,
            camera_frame=camera_frame,
            tf_stamp=msg.header.stamp,
        )

        detection_msg = ConeDetectionStamped()
        detection_msg.header.stamp = msg.header.stamp
        detection_msg.header.frame_id = self.output_frame
        detection_msg.cones = cones
        detection_msg.cones_with_cov = cones_cov
        self.detection_pub.publish(detection_msg)

        if self.annotated_pub is not None:
            self.annotated_pub.publish(
                self._annotate(msg, bgr, detections, len(cones))
            )

        self.fps.next_iter()
        self.get_logger().info(
            f"raw_detections={len(detections)} cones={len(cones)} | "
            f"infer: {infer_ms:.1f}ms | FPS: {self.fps.fps():.1f}",
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------ #
    # debug image
    # ------------------------------------------------------------------ #
    def _annotate(self, msg, bgr, detections, n_cones_3d: int) -> Image:
        """Raw 2D boxes + confidence, for the operator to verify YOLO live."""
        out = bgr.copy()
        for x1, y1, x2, y2, conf, cls in detections:
            color = _CLASS_BGR.get(int(cls), (200, 200, 200))
            cv2.rectangle(
                out, (int(x1), int(y1)), (int(x2), int(y2)), color, 2
            )
            label = f"{_CLASS_LABEL.get(int(cls), '?')} {conf:.2f}"
            baseline_y = int(y1) - 6 if int(y1) > 24 else int(y1) + 20
            cv2.putText(
                out, label, (int(x1), baseline_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
            )
        cv2.putText(
            out,
            f"YOLO {len(detections)} det / {n_cones_3d} 3D",
            (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
            cv2.LINE_AA,
        )
        annotated = self.cv_bridge.cv2_to_imgmsg(out, "bgr8")
        annotated.header = msg.header
        return annotated

    # ------------------------------------------------------------------ #
    # 3D back-projection + TF
    # ------------------------------------------------------------------ #
    def _build_cones(
        self,
        detections: List[Tuple[float, float, float, float, float, int]],
        camera_info: CameraInfo,
        depth: np.ndarray,
        camera_frame: str,
        tf_stamp=None,
    ) -> Tuple[List[Cone], List[ConeWithCovariance]]:
        """Convert 2D detections into 3D cones in the output frame."""
        cones: List[Cone] = []
        cones_cov: List[ConeWithCovariance] = []

        if not detections:
            return cones, cones_cov

        K = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx0, cy0 = K[0, 2], K[1, 2]

        img_h, img_w = depth.shape[:2]

        # Look up transform camera_frame -> output_frame once per frame.
        transform = self._lookup_transform(camera_frame, tf_stamp)
        if transform is None:
            return cones, cones_cov

        for x1, y1, x2, y2, conf, cls in detections:
            # Sample depth at the centre of the bounding box.
            cx_px = int(round((x1 + x2) / 2.0))
            cy_px = int(round((y1 + y2) / 2.0))
            cx_px = int(np.clip(cx_px, 0, img_w - 1))
            cy_px = int(np.clip(cy_px, 0, img_h - 1))

            # Apparent pixel height of the cone bounding box.
            bbox_h = max(1.0, float(y2 - y1))

            # Sample depth over a small window around the centre (robust to holes).
            win = 5
            x0 = max(0, cx_px - win)
            x1b = min(img_w, cx_px + win + 1)
            y0 = max(0, cy_px - win)
            y1b = min(img_h, cy_px + win + 1)
            patch = depth[y0:y1b, x0:x1b]
            valid = patch[np.isfinite(patch) & (patch > 0.0)]
            if valid.size == 0:
                z = (fy * self.cone_real_height) / bbox_h
                z = float(np.clip(z, self.min_range, self.max_range))
            else:
                z = float(np.median(valid))
                if z < self.min_range or z > self.max_range:
                    z = (fy * self.cone_real_height) / bbox_h
                    z = float(np.clip(z, self.min_range, self.max_range))

            # Back-project to camera optical frame (X right, Y down, Z forward).
            x_opt = (cx_px - cx0) * z / fx
            y_opt = (cy_px - cy0) * z / fy
            z_opt = z

            # The depth image header frame is the camera body frame
            # (X forward, Y left, Z up). Convert optical coordinates to body
            # coordinates before applying the body->output transform.
            x_cam = z_opt           # forward
            y_cam = -x_opt          # left
            z_cam = -y_opt          # up

            # Transform to output frame.
            x_out, y_out, z_out = self._apply_transform(
                x_cam, y_cam, z_cam, transform
            )

            color = CLASS_TO_COLOR.get(int(cls), Cone.UNKNOWN)

            cone = Cone()
            cone.location = Point(x=float(x_out), y=float(y_out), z=float(z_out))
            cone.color = color
            cones.append(cone)

            cone_cov = ConeWithCovariance()
            cone_cov.cone = cone
            # Simple diagonal covariance (depth noise + angular noise).
            range_std = max(0.05, 0.01 * z)
            cone_cov.covariance = [
                range_std * range_std, 0.0,
                0.0, range_std * range_std,
            ]
            cones_cov.append(cone_cov)

        return cones, cones_cov

    def _lookup_transform(
        self, camera_frame: str, stamp=None
    ) -> Optional[TransformStamped]:
        from rclpy.duration import Duration

        # Look the transform up at the image's own timestamp. Looking it up at
        # "now" applies the car's *current* pose to an image taken tens of ms
        # ago, which smears cone positions by v*dt while driving and splits the
        # same physical cone into several map entries (ghost cones).
        when = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
        try:
            return self.tf_buffer.lookup_transform(
                self.output_frame, camera_frame, when,
                timeout=Duration(seconds=0.1)
            )
        except TransformException:
            pass
        # The exact-stamp lookup can fail right after startup (buffer not yet
        # populated at that instant); falling back to the latest transform is
        # better than dropping the frame, but flag it: positions may smear.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, camera_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            self.get_logger().warn(
                "TF at image stamp unavailable, using latest (motion smear possible)",
                throttle_duration_sec=5.0,
            )
            return tf
        except TransformException:
            self.get_logger().warn(
                f"TF {self.output_frame}<-{camera_frame} unavailable, skipping frame",
                throttle_duration_sec=2.0,
            )
            return None

    @staticmethod
    def _apply_transform(
        x: float, y: float, z: float, tf: TransformStamped
    ) -> Tuple[float, float, float]:
        """Apply a rigid transform (translation + quaternion rotation)."""
        t = tf.transform.translation
        q = tf.transform.rotation

        # Quaternion rotation: p' = q * p * q^-1
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        # Rotate point
        # (qw^2 - qx^2 - qy^2 - qz^2) simplifies via the standard formula:
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z

        x_out = ix * qw + iw * -qx + iy * -qz - iz * -qy
        y_out = iy * qw + iw * -qy + iz * -qx - ix * -qz
        z_out = iz * qw + iw * -qz + ix * -qy - iy * -qx

        return (x_out + t.x, y_out + t.y, z_out + t.z)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    # SingleThreadedExecutor: CUDA kernels launched from the same thread
    # that loaded the model avoid the multi-second synchronize stalls seen
    # with MultiThreadedExecutor (model loaded in __init__ on the main
    # thread but detect() invoked from a worker thread).
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()