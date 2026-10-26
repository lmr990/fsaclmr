"""Integration monitor for the YOLO pipeline while the simulation runs.

Measures over a window:
- /zed2i/.../image_rect_color publish rate
- /slam/cone_detection output rate + message age (against /clock, sim time)
- CPU/RSS of yolo_detector_node and gzserver, GPU memory/util snapshots

Prints a JSON summary at the end. Run alongside `ros2 launch ...yolo...`.
"""

import json
import os
import subprocess
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rosgraph_msgs.msg import Clock
from driverless_msgs.msg import ConeDetectionStamped
from sensor_msgs.msg import Image

DURATION = float(os.environ.get("MONITOR_S", "45"))


def proc_stats(pid: int):
    try:
        with open(f"/proc/{pid}/stat") as f:
            parts = f.read().split()
        utime, stime = int(parts[13]), int(parts[14])
        rss_kb = int(parts[23]) * 4 // 1024
        with open(f"/proc/{pid}/cmdline") as f:
            cmd = f.read().replace("\0", " ")[:60]
        return {"cpu_ticks": utime + stime, "rss_mb": rss_kb, "cmd": cmd}
    except Exception:
        return None


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__("yolo_monitor")
        self.t0 = time.monotonic()
        self.img_times: deque = deque(maxlen=2000)
        self.det_times: deque = deque(maxlen=2000)
        self.det_ages: deque = deque(maxlen=2000)
        self.clock = 0.0
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, "/zed2i/zed_node/rgb/image_rect_color",
                                 lambda m: self.img_times.append(time.monotonic()), qos)
        self.create_subscription(ConeDetectionStamped, "/slam/cone_detection", self._det, qos)
        self.create_subscription(Clock, "/clock", lambda m: setattr(self, "clock", m.clock.sec + m.clock.nanosec * 1e-9), qos)

    def _det(self, msg: ConeDetectionStamped) -> None:
        self.det_times.append(time.monotonic())
        if self.clock > 0:
            self.det_ages.append(self.clock - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))


def find_pid(pattern: str):
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline") as f:
                if pattern in f.read():
                    return int(pid)
        except Exception:
            pass
    return None


def pct(a, q):
    return round(float(__import__("numpy").percentile(a, q)), 2) if len(a) else None


def main() -> None:
    rclpy.init()
    node = Monitor()
    yolo_pid = find_pid("yolo_detector_node")
    _g = subprocess.run(["pgrep", "-x", "gzserver"], capture_output=True, text=True).stdout.split()
    gz_pid = int(_g[0]) if _g else None
    samples = []

    def sampler():
        while time.monotonic() - node.t0 < DURATION:
            s = {"t": time.monotonic() - node.t0}
            for name, pid in (("yolo", yolo_pid), ("gzserver", gz_pid)):
                st = proc_stats(pid) if pid else None
                if st:
                    s[name] = st
            gpu = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,pstate,clocks.sm",
                                  "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout.strip()
            if gpu:
                s["gpu"] = gpu
            samples.append(s)
            time.sleep(2)

    th = threading.Thread(target=sampler, daemon=True)
    th.start()
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    time.sleep(DURATION)
    wall = time.monotonic() - node.t0

    def rate(d):
        if len(d) < 2:
            return None
        return round((len(d) - 1) / (d[-1] - d[0]), 2)

    y0 = samples[0] if samples else {}
    yl = samples[-1] if samples else {}
    cpu = {}
    for name in ("yolo", "gzserver"):
        if name in y0 and name in yl and "cpu_ticks" in y0[name]:
            hz = os.sysconf("SC_CLK_TCK")
            cpu[name] = round((yl[name]["cpu_ticks"] - y0[name]["cpu_ticks"]) / hz / wall * 100, 1)
            cpu[name + "_rss_mb"] = yl[name]["rss_mb"]

    result = {
        "window_s": round(wall, 1),
        "camera_fps": rate(node.img_times),
        "detection_fps": rate(node.det_times),
        "detections_count_avg": round(float(sum(len(m.cones) for m in [])[0:0] or 0), 1) if False else None,
        "det_age_s_avg": pct(list(node.det_ages), 50),
        "det_age_s_p95": pct(list(node.det_ages), 95),
        "cpu_percent": cpu,
        "gpu_first": y0.get("gpu"), "gpu_last": yl.get("gpu"),
        "n_samples": len(samples),
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    os.makedirs(os.path.expanduser("~/qutms_ws/diagnostics"), exist_ok=True)
    tag = os.environ.get("MONITOR_TAG", "run")
    with open(os.path.expanduser(f"~/qutms_ws/diagnostics/monitor_{tag}.json"), "w") as f:
        json.dump(result, f, indent=2)
    os._exit(0)


if __name__ == "__main__":
    main()
