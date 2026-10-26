"""YOLO cone detection ROS2 node.

Subscribes to a stereo camera's RGB image, registered depth image and
camera info, runs TCone-YOLO inference, back-projects detections into 3D
using the depth map, transforms them into the output frame (default
``track``) and publishes a ``driverless_msgs/ConeDetectionStamped`` message
to ``slam/cone_detection`` so the path planner can consume it directly.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

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
        if not tcone_yolo_path or not Path(tcone_yolo_path).exists():
            self.get_logger().fatal(f"TCone-YOLO path not found: {tcone_yolo_path}")
            raise FileNotFoundError(f"TCone-YOLO path not found: {tcone_yolo_path}")
        tcone_root = Path(tcone_yolo_path).resolve()

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
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_depth_frame: str = ""

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
            self._latest_depth = depth
            self._latest_depth_frame = msg.header.frame_id
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert depth image: {exc}", throttle_duration_sec=2.0)

    def _image_cb(self, msg: Image) -> None:
        if self._latest_camera_info is None:
            self.get_logger().warn("Camera info not yet received, skipping frame", throttle_duration_sec=2.0)
            return
        if self._latest_depth is None:
            self.get_logger().warn("Depth image not yet received, skipping frame", throttle_duration_sec=2.0)
            return

        try:
            bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to convert RGB image: {exc}", throttle_duration_sec=2.0)
            return

        camera_frame = msg.header.frame_id or self._latest_depth_frame
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
            depth=self._latest_depth,
            camera_frame=camera_frame,
        )

        detection_msg = ConeDetectionStamped()
        detection_msg.header.stamp = msg.header.stamp
        detection_msg.header.frame_id = self.output_frame
        detection_msg.cones = cones
        detection_msg.cones_with_cov = cones_cov
        self.detection_pub.publish(detection_msg)

        self.fps.next_iter()
        self.get_logger().info(
            f"raw_detections={len(detections)} cones={len(cones)} | "
            f"infer: {infer_ms:.1f}ms | FPS: {self.fps.fps():.1f}",
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------ #
    # 3D back-projection + TF
    # ------------------------------------------------------------------ #
    def _build_cones(
        self,
        detections: List[Tuple[float, float, float, float, float, int]],
        camera_info: CameraInfo,
        depth: np.ndarray,
        camera_frame: str,
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
        transform = self._lookup_transform(camera_frame)
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
                z = float(np.clip(z, self.min_range, self.max_range * 2.0))
            else:
                z = float(np.median(valid))
                if z < self.min_range or z > self.max_range:
                    z = (fy * self.cone_real_height) / bbox_h
                    z = float(np.clip(z, self.min_range, self.max_range * 2.0))

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

    def _lookup_transform(self, camera_frame: str) -> Optional[TransformStamped]:
        from rclpy.duration import Duration
        try:
            return self.tf_buffer.lookup_transform(
                self.output_frame, camera_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
        except TransformException:
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
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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