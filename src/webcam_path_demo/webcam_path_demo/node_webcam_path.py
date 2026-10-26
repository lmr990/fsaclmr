"""Webcam cone detection + planned-path overlay demo.

Pipeline (static camera):
    webcam frame -> YOLO cones -> monocular distance -> ConeDetectionStamped
    -> (cone_mapper_node -> ft_planner_node, launched alongside)
    -> /planning/midline_path -> back-projected onto the image -> overlay.

Distance models:
    "size"   : z = fy * cone_real_height / bbox_pixel_height   (fallback in
               the sim node too; good enough for a demo)
    "ground" : bottom-center pixel + camera height + downward pitch solved
               against the ground plane (better lateral/range accuracy)

The camera frame IS the world frame ("track"): the car sits at the origin,
so no odometry/TF beyond the static identity TF published in the launch.
"""

import glob
import os
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from driverless_msgs.msg import Cone, ConeDetectionStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from yolo_cone_detection.yolo_inference import YoloInference

# Same convention as the sim node: red/orange are merged into small orange.
CLASS_TO_COLOR = {
    0: Cone.ORANGE_SMALL,
    1: Cone.BLUE,
    2: Cone.YELLOW,
}

COLOR_BGR = {
    Cone.ORANGE_SMALL: (0, 100, 255),
    Cone.BLUE: (255, 100, 0),
    Cone.YELLOW: (0, 220, 255),
    Cone.UNKNOWN: (160, 160, 160),
}


class WebcamPathNode(Node):
    def __init__(self) -> None:
        super().__init__("webcam_path_node")

        # ---- source ----
        self.declare_parameter("source", "camera")           # "camera" | "file" | "topic"
        self.declare_parameter("camera_device", 0)
        self.declare_parameter("image_files", "")            # glob for "file" mode
        self.declare_parameter("loop_files", True)
        self.declare_parameter("fps_limit", 15.0)
        # "topic" mode: real ZED via the ROS wrapper (v5 topic names)
        self.declare_parameter("topic_image", "/zed2i/zed_node/rgb/color/rect/image")
        self.declare_parameter("topic_depth", "/zed2i/zed_node/depth/depth_registered")
        self.declare_parameter("topic_info", "/zed2i/zed_node/rgb/color/rect/image/camera_info")

        # ---- YOLO (same semantics as yolo_detector_node) ----
        self.declare_parameter("tcone_yolo_path", "")
        self.declare_parameter("weights", "")
        self.declare_parameter("data_yaml", "")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.15)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 100)
        self.declare_parameter("device", "0")
        self.declare_parameter("half", False)

        # ---- monocular geometry ----
        self.declare_parameter("distance_method", "size")    # "size" | "ground"
        self.declare_parameter("cone_real_height", 0.325)    # FSAC small cone [m]
        self.declare_parameter("camera_height", 0.35)        # lens height above ground [m]
        self.declare_parameter("camera_pitch_deg", 15.0)     # downward tilt [deg]
        self.declare_parameter("intrinsics_file", "")        # yaml with fx,fy,cx,cy
        self.declare_parameter("fov_h_deg", 65.0)            # fallback estimate
        self.declare_parameter("min_range", 0.4)
        self.declare_parameter("max_range", 12.0)

        # ---- io ----
        self.declare_parameter("publish_topic", "slam/cone_detection")
        self.declare_parameter("frame_id", "track")
        self.declare_parameter("path_topic", "/planning/midline_path")
        self.declare_parameter("annotated_topic", "/webcam/annotated_image")
        self.declare_parameter("display", True)
        self.declare_parameter("save_debug_png", "")         # if set, save latest overlay

        self.p = {k: self.get_parameter(k).value for k in [
            "source", "camera_device", "image_files", "loop_files", "fps_limit",
            "topic_image", "topic_depth", "topic_info",
            "tcone_yolo_path", "weights", "data_yaml", "imgsz", "conf_thres",
            "iou_thres", "max_det", "device", "half",
            "distance_method", "cone_real_height", "camera_height",
            "camera_pitch_deg", "intrinsics_file", "fov_h_deg",
            "min_range", "max_range",
            "publish_topic", "frame_id", "path_topic", "annotated_topic",
            "display", "save_debug_png"]}

        self.bridge = CvBridge()
        self.K: Optional[np.ndarray] = None                  # 3x3 intrinsics
        self.frame_size: Optional[Tuple[int, int]] = None    # (w, h)
        self.cap = None
        self.files: List[str] = []
        self.file_idx = 0
        self.latest_dets = []                                # for overlay reuse
        self.latest_path: Optional[Path] = None
        self._detect_lock = threading.Lock()

        # ---- inference (reuses the project's exact wrapper) ----
        base = os.path.expanduser("~/qutms_ws/src/TCone-YOLO-main")
        weights = self.p["weights"] or os.path.join(
            base, "compared-results/model/TCone-YOLO_model/weights/best.pt")
        data_yaml = self.p["data_yaml"] or os.path.join(base, "data/Cone.yaml")
        self.get_logger().info("Loading YOLO model (this may take a few seconds)...")
        self.inference = YoloInference(
            weights=weights, data_yaml=data_yaml,
            tcone_root=self.p["tcone_yolo_path"] or base,
            imgsz=int(self.p["imgsz"]), conf_thres=float(self.p["conf_thres"]),
            iou_thres=float(self.p["iou_thres"]), max_det=int(self.p["max_det"]),
            device=str(self.p["device"]), half=bool(self.p["half"]),
        )
        self.inference.load()
        self.get_logger().info("YOLO model ready")

        # ---- ros io ----
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.cone_pub = self.create_publisher(ConeDetectionStamped, self.p["publish_topic"], 10)
        self.img_pub = self.create_publisher(Image, self.p["annotated_topic"], qos)
        self.create_subscription(Path, self.p["path_topic"], self._path_cb, 10)
        if self.p["source"] in ("camera", "topic") and not self.p["intrinsics_file"]:
            # allow camera_info from the source to refine intrinsics
            info_topic = (self.p["topic_info"] if self.p["source"] == "topic"
                          else "camera_info")
            self.create_subscription(CameraInfo, info_topic, self._info_cb, qos)

        self._topic_frame = None
        self._topic_depth = None
        self._frame_evt = threading.Event()
        self._init_source()
        self._resolve_intrinsics()

        # capture thread + ros spin in background, display in main thread
        self._stop = threading.Event()
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

        period = 1.0 / max(5.0, float(self.p["fps_limit"]))
        self.create_timer(period, self._draw_loop)

    # ------------------------------------------------------------------ #
    # source / intrinsics
    # ------------------------------------------------------------------ #
    def _init_source(self) -> None:
        if self.p["source"] == "camera":
            dev = int(self.p["camera_device"])
            self.cap = cv2.VideoCapture(dev)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera {dev}")
            self.get_logger().info(f"Camera {dev} opened")
        elif self.p["source"] == "topic":
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(Image, self.p["topic_image"], self._topic_rgb_cb, qos)
            self.create_subscription(Image, self.p["topic_depth"], self._topic_depth_cb, qos)
            self.get_logger().info(
                f"Topic source: {self.p['topic_image']} + {self.p['topic_depth']}")
        else:
            pattern = self.p["image_files"] or os.path.expanduser(
                "~/qutms_ws/diagnostics/frames/rgb_*.png")
            self.files = sorted(glob.glob(os.path.expanduser(pattern)))
            if not self.files:
                raise RuntimeError(f"No images match: {pattern}")
            self.get_logger().info(f"File source: {len(self.files)} images")

    def _topic_rgb_cb(self, msg: Image) -> None:
        self._topic_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self._frame_evt.set()

    def _topic_depth_cb(self, msg: Image) -> None:
        self._topic_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def _resolve_intrinsics(self) -> None:
        if self.p["intrinsics_file"]:
            import yaml
            with open(os.path.expanduser(self.p["intrinsics_file"])) as f:
                cal = yaml.safe_load(f)
            self.K = np.array(cal["K"], dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f"K from {self.p['intrinsics_file']}")
        # else: estimate on first frame (FOV) or camera_info when available

    def _ensure_K(self, frame) -> None:
        if self.K is not None:
            return
        h, w = frame.shape[:2]
        if self._info_K is not None and self.p["source"] in ("camera", "topic") \
                and self._info_size == (w, h):
            self.K = self._info_K
            self.get_logger().info("K from camera_info")
        else:
            fov = np.deg2rad(float(self.p["fov_h_deg"]))
            fx = w / (2.0 * np.tan(fov / 2.0))
            self.K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]])
            self.get_logger().warn(
                f"K estimated from FOV={self.p['fov_h_deg']}deg (fx={fx:.0f}) — "
                "run webcam_calibrate for accurate distances")
        self.frame_size = (w, h)

    _info_K = None
    _info_size = None

    def _info_cb(self, msg: CameraInfo) -> None:
        if self._info_K is None:
            self._info_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._info_size = (msg.width, msg.height)

    # ------------------------------------------------------------------ #
    # geometry
    # ------------------------------------------------------------------ #
    def _distance_size(self, bbox: Tuple[float, float, float, float]) -> float:
        x1, y1, x2, y2 = bbox
        h_px = max(1.0, y2 - y1)
        z = self.K[1, 1] * float(self.p["cone_real_height"]) / h_px
        return float(np.clip(z, self.p["min_range"], self.p["max_range"] * 2.0))

    def _u_to_lateral(self, u: float, z: float) -> float:
        """Image column -> track lateral offset. REP-103: +y is LEFT, so a
        cone on the image right (u > cx) gets negative y (same convention as
        the sim node's optical->body conversion)."""
        return (self.K[0, 2] - u) * z / self.K[0, 0]

    def _distance_ground(self, u: float, v: float) -> float:
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        h = float(self.p["camera_height"])
        th = np.deg2rad(float(self.p["camera_pitch_deg"]))
        # camera-frame ray: x fwd, y left, z up
        d = np.array([1.0, (u - cx) / fx, -(v - cy) / fy])
        f = np.array([np.cos(th), 0.0, -np.sin(th)])
        l = np.array([0.0, 1.0, 0.0])
        up = np.array([np.sin(th), 0.0, np.cos(th)])
        dw = d[0] * f + d[1] * l + d[2] * up
        if dw[2] >= -1e-6:               # ray pointing at/above horizon
            return float(self.p["max_range"])
        t = -h / dw[2]                   # camera at height h, ground z=0
        X = t * dw[0]
        return float(np.clip(X, self.p["min_range"], self.p["max_range"] * 2.0))

    def _cone_from_detection(self, det) -> Tuple[Cone, Tuple[float, float]]:
        x1, y1, x2, y2, conf, cls = det
        u = (x1 + x2) / 2.0

        # Real depth (topic mode with ZED) takes priority; fall back to the
        # monocular models when the depth lookup is invalid.
        z = None
        depth = self._topic_depth if self.p["source"] == "topic" else None
        if depth is not None:
            dh, dw = depth.shape[:2]
            px = int(np.clip(round(u), 0, dw - 1))
            py = int(np.clip(round(y2), 0, dh - 1))   # cone base sits on the ground
            win = 5
            patch = depth[max(0, py - win):py + win + 1, max(0, px - win):px + win + 1]
            valid = patch[np.isfinite(patch) & (patch > 0.05)]
            if valid.size:
                cand = float(np.median(valid))
                if self.p["min_range"] <= cand <= self.p["max_range"]:
                    z = cand
        if z is None:
            if self.p["distance_method"] == "ground":
                z = self._distance_ground(u, y2)      # bottom-center on ground
            else:
                z = self._distance_size((x1, y1, x2, y2))
        lateral = self._u_to_lateral(u, z)
        cone = Cone()
        cone.location = Point(x=float(z), y=float(lateral), z=0.0)
        cone.color = CLASS_TO_COLOR.get(int(cls), Cone.UNKNOWN)
        return cone, (u, y2)

    # ------------------------------------------------------------------ #
    # capture + detect + publish
    # ------------------------------------------------------------------ #
    def _read_frame(self):
        if self.p["source"] == "camera":
            ok, frame = self.cap.read()
            return frame if ok else None
        if self.p["source"] == "topic":
            if not self._frame_evt.wait(timeout=1.0):
                return None
            self._frame_evt.clear()
            return self._topic_frame
        if not self.files:
            return None
        path = self.files[self.file_idx]
        self.file_idx += 1
        if self.file_idx >= len(self.files):
            if not self.p["loop_files"]:
                self._stop.set()
            self.file_idx = 0
        return cv2.imread(path)

    def _capture_loop(self) -> None:
        last = 0.0
        period = 1.0 / max(1.0, float(self.p["fps_limit"]))
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            if now - last < period:
                time.sleep(0.01)
                continue
            frame = self._read_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            self._ensure_K(frame)
            t0 = time.perf_counter()
            dets = self.inference.detect(frame)
            dt = (time.perf_counter() - t0) * 1000.0

            cones, anchors = [], []
            for d in dets:
                cone, anchor = self._cone_from_detection(d)
                cones.append(cone)
                anchors.append(anchor)

            msg = ConeDetectionStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p["frame_id"]
            msg.cones = cones
            self.cone_pub.publish(msg)

            with self._detect_lock:
                self._frame = frame
                self._dets = dets
                self._anchors = anchors
                self._detect_ms = dt
            last = now

    # ------------------------------------------------------------------ #
    # overlay + display
    # ------------------------------------------------------------------ #
    def _path_cb(self, msg: Path) -> None:
        self.latest_path = msg

    def _image_project(self, X: float, Y: float, assume_cone_center: bool = False) -> Optional[Tuple[int, int]]:
        """Track-frame ground point (X fwd, Y left) -> pixel. Inverse of the
        active distance model."""
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        if self.p["distance_method"] == "ground":
            th = np.deg2rad(float(self.p["camera_pitch_deg"]))
            h = float(self.p["camera_height"])
            f = np.array([np.cos(th), 0.0, -np.sin(th)])
            l = np.array([0.0, -1.0, 0.0])            # image right = world -y
            up = np.array([np.sin(th), 0.0, np.cos(th)])
            v = np.array([X, Y, -h])                     # world -> camera pos
            xc, yc, zc = v @ f, v @ l, v @ up
            if xc <= 1e-3:
                return None
            return int(cx + fx * (-yc) / xc), int(cy + fy * (-zc) / xc)
        # size model: cone center at height H/2... use bottom-at-ground trick:
        # a point at distance X on the ground under a cone of height H sits at
        # v = cy + fy*H/X (same relation as the forward model).
        if X <= 0.2:
            return None
        v = cy + fy * float(self.p["cone_real_height"]) / X
        u = cx - fx * Y / X                              # +y left -> image left
        return int(u), int(v)

    def _draw_loop(self) -> None:
        with self._detect_lock:
            frame = getattr(self, "_frame", None)
            dets = getattr(self, "_dets", [])
            anchors = getattr(self, "_anchors", [])
            dms = getattr(self, "_detect_ms", 0.0)
        if frame is None:
            return

        out = frame.copy()
        for (x1, y1, x2, y2, conf, cls) in dets:
            color = COLOR_BGR[CLASS_TO_COLOR.get(int(cls), Cone.UNKNOWN)]
            cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(out, f"{conf:.2f}", (int(x1), int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        for (u, v) in anchors:
            cv2.circle(out, (int(u), int(v)), 4, (0, 255, 0), -1)

        path = self.latest_path
        pts = []
        if path is not None:
            for ps in path.poses:
                p = self._image_project(ps.pose.position.x, ps.pose.position.y)
                if p is not None:
                    pts.append(p)
        for a, b in zip(pts, pts[1:]):
            cv2.line(out, a, b, (0, 255, 0), 3)
        for p in pts[::5]:
            cv2.circle(out, p, 3, (0, 255, 255), -1)

        depth_src = "zed" if (self.p["source"] == "topic"
                              and getattr(self, "_topic_depth", None) is not None) else "est"
        cv2.putText(out, f"cones={len(dets)} detect={dms:.0f}ms depth={depth_src} path={'yes' if pts else 'no'}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        if self.p["save_debug_png"]:
            cv2.imwrite(os.path.expanduser(self.p["save_debug_png"]), out)

        if self.p["display"]:
            cv2.imshow("webcam path demo", out)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise SystemExit(0)

    def shutdown(self) -> None:
        self._stop.set()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamPathNode()
    try:
        while rclpy.ok():
            time.sleep(0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
