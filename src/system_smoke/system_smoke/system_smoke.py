#!/usr/bin/env python3
"""System integration smoke (governance phase 2).

External seam checker: after a chain is started, verifies from OUTSIDE that
the seams hold. No project node is modified or introspected beyond the
standard ROS graph/param/TF interfaces. Prints one PASS/FAIL line per check
and exits non-zero when any check fails.

Checks (per profile, values from the profile file):
  tf       — required edges exist; camera mount translation within 0.01 m and
             rotation within 0.5 deg of the profile expectation (F2); leaf
             camera frames resolvable (F3: wrapper publish_urdf lifeline).
  liveness — every key topic receives >=1 message within the window and its
             frame_id matches the profile whitelist (F10).
  qos      — for each key edge, publisher/subscriber reliability combinations
             are deliverable: BEST_EFFORT pub + RELIABLE sub is the fatal
             mismatch (F9).
  params   — live parameter values via the nodes' param API (not files):
             wheelbase must equal the URDF truth, size gate and cone heights
             must match the detector contract, expected nodes must be running.

Profiles ship in this package's config/ (governance phase 3 moves the
environment values to vehicle_bringup/config/profiles/ and this tool reads
them from there).

Usage:
  ros2 run system_smoke system_smoke --profile sim
  ros2 run system_smoke system_smoke --profile real --only tf,liveness
  ros2 run system_smoke system_smoke --profile-file /tmp/mini.yaml
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory

# F2 tolerance: mount translation [m] / rotation [deg]
MOUNT_TOL_M = 0.01
MOUNT_TOL_DEG = 0.5


def quat_to_rpy(x: float, y: float, z: float, w: float):
    """Quaternion -> (roll, pitch, yaw) radians (Z-Y-X convention)."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class CheckResult:
    def __init__(self, group: str, ok: bool, detail: str, skipped: bool = False):
        self.group, self.ok, self.detail, self.skipped = group, ok, detail, skipped

    def line(self) -> str:
        if self.skipped:
            tag = "SKIP"
        elif self.ok:
            tag = "PASS"
        else:
            tag = "FAIL"
        return f"{tag:4s} [{self.group:8s}] {self.detail}"


def load_profile(args) -> dict:
    """Profile source: --profile-file, else this package's config/<profile>.yaml."""
    import yaml

    if args.profile_file:
        return yaml.safe_load(Path(args.profile_file).read_text())
    cfg = Path(get_package_share_directory("system_smoke")) / "config" / f"{args.profile}.yaml"
    if not cfg.exists():
        raise SystemExit(f"profile config not found: {cfg}")
    return yaml.safe_load(cfg.read_text())


def sh(cmd: list, timeout: float = 10.0) -> str:
    """Run a ROS CLI command and return stdout (empty on failure)."""
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return out.stdout
    except (subprocess.TimeoutExpired, OSError):
        return ""


def _wait_tf(node: Node, parent: str, child: str, timeout: float):
    """Spin the node while waiting for a transform (lookup's own timeout does
    not service the executor, so the /tf_static listener never fires)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.tf_buffer.can_transform(parent, child, rclpy.time.Time()):
            return node.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
    return None


def check_tf(node: Node, prof: dict) -> list:
    results = []
    tf = prof.get("tf", {})
    timeout = float(tf.get("wait_sec", 3.0))
    for edge in tf.get("edges", []):
        parent, child = edge["parent"], edge["child"]
        t = _wait_tf(node, parent, child, timeout)
        if t is None:
            results.append(CheckResult("tf", False, f"edge {parent}->{child} NOT resolvable within {timeout}s"))
            continue
        note = edge.get("note", "")
        if not edge.get("check_values", False):
            results.append(CheckResult("tf", True, f"edge {parent}->{child} ok ({note})" if note else f"edge {parent}->{child} ok"))
            continue
        # mount value check (F2)
        tr = t.transform.translation
        q = t.transform.rotation
        r, p, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        exp = edge.get("expect", {})
        bad = []
        for axis, got, want in (("x", tr.x, exp.get("x")), ("y", tr.y, exp.get("y")), ("z", tr.z, exp.get("z")),
                                ("roll", r, exp.get("roll")), ("pitch", p, exp.get("pitch")), ("yaw", y, exp.get("yaw"))):
            if want is None:
                continue
            diff = abs(got - float(want))
            tol = MOUNT_TOL_DEG * math.pi / 180 if axis in ("roll", "pitch", "yaw") else MOUNT_TOL_M
            if diff > tol:
                bad.append(f"{axis}={got:+.3f} expect {want:+.3f} (Δ{diff:.3f})")
        if bad:
            results.append(CheckResult("tf", False, f"mount {parent}->{child} VALUE MISMATCH: " + "; ".join(bad)))
        else:
            results.append(CheckResult("tf", True, f"mount {parent}->{child} values within tolerance "
                                       f"(±{MOUNT_TOL_M} m / ±{MOUNT_TOL_DEG} deg)"))
    for leaf in tf.get("leaf_frames", []):
        root = tf.get("root", tf["edges"][0]["parent"])
        ok = _wait_tf(node, root, leaf, timeout) is not None
        results.append(CheckResult("tf", ok, f"camera leaf frame '{leaf}' resolvable"
                                   + ("" if ok else " (wrapper publish_urdf off? see F3)")))
    return results


def check_liveness(node: Node, prof: dict, window: float) -> list:
    results = []
    sub_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
    from rosidl_runtime_py.utilities import get_message

    seen: dict = {}
    for item in prof.get("liveness", []):
        topic = item["topic"]
        seen[topic] = {"n": 0, "frame": None}
        msg_cls = get_message(item["msg_type"])

        def make_cb(t):
            def cb(msg):
                seen[t]["n"] += 1
                seen[t]["frame"] = getattr(getattr(msg, "header", None), "frame_id", None)
            return cb
        node.create_subscription(msg_cls, topic, make_cb(topic), sub_qos)

    end = time.monotonic() + window
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for item in prof.get("liveness", []):
        topic, frames = item["topic"], item.get("frames", [])
        s = seen[topic]
        if s["n"] == 0:
            results.append(CheckResult("liveness", False, f"{topic}: 0 messages in {window:.0f}s"))
        elif frames and (s["frame"] is None or not any(f in s["frame"] for f in frames)):
            results.append(CheckResult("liveness", False,
                                       f"{topic}: {s['n']} msgs but frame_id '{s['frame']}' not in {frames} (F10)"))
        else:
            results.append(CheckResult("liveness", True, f"{topic}: {s['n']} msgs, frame '{s['frame']}'"))
    return results


def check_qos(prof: dict) -> list:
    """ros2 topic info -v: fatal mismatch = BEST_EFFORT pub + RELIABLE sub (F9)."""
    results = []
    for topic in prof.get("qos_topics", []):
        out = sh(["ros2", "topic", "info", "-v", topic])
        if "Publisher count: 0" in out or (out.strip() and "Publisher count" not in out):
            results.append(CheckResult("qos", True, f"{topic}: no publisher (skip)"))
            continue
        section = None
        pub_rel, sub_rel = [], []
        for ln in out.splitlines():
            if ln.strip().startswith("Publisher count:"):
                section = "pub"
            elif ln.strip().startswith("Subscription count:"):
                section = "sub"
            elif "Reliability:" in ln:
                val = ln.split(":")[1].strip()
                (pub_rel if section == "pub" else sub_rel).append(val)
        if any(p == "BEST_EFFORT" for p in pub_rel) and any(s == "RELIABLE" for s in sub_rel):
            results.append(CheckResult("qos", False,
                                       f"{topic}: BEST_EFFORT publisher with RELIABLE subscriber -> sub receives nothing (F9)"))
        else:
            results.append(CheckResult("qos", True,
                                       f"{topic}: pub={set(pub_rel) or 'none'} sub={set(sub_rel) or 'none'} deliverable"))
    return results


def check_params(node: Node, prof: dict) -> list:
    """Live param reads via each node's parameter service (rclpy graph API,
    no ros2 CLI daemon: its discovery cache races freshly started nodes)."""
    results = []
    want_nodes = set(prof.get("nodes", []))
    if want_nodes:
        # brief spin so the graph discovery populates
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Humble returns (name, namespace) tuples from the graph API
        running = [f"{ns.rstrip('/')}/{nm}" for nm, ns in node.get_node_names_and_namespaces()]
        missing = [n for n in want_nodes if not any(r.endswith(f"/{n}") for r in running)]
        if missing:
            results.append(CheckResult("params", False, f"expected nodes not running: {missing}"))
        else:
            results.append(CheckResult("params", True, f"expected nodes present ({len(want_nodes)})"))
    from rcl_interfaces.srv import GetParameters
    for item in prof.get("params", []):
        fqn, name = item["node"].rstrip("/"), item["name"].lstrip("/")
        try:
            client = node.create_client(GetParameters, f"{fqn}/get_parameters")
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("parameter service not reachable")
            req = GetParameters.Request()
            req.names = [name]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            node.destroy_client(client)
            resp = future.result()
            if resp is None or not resp.values or resp.values[0].type == 0:
                results.append(CheckResult("params", False, f"{fqn} {name}: param not set on node"))
                continue
            v = resp.values[0]
            # rcl_interfaces ParameterType: 1 BOOL, 2 INTEGER, 3 DOUBLE,
            # 4 STRING, 6 BOOL_ARRAY, 7 INTEGER_ARRAY, 8 DOUBLE_ARRAY
            got = {1: str(v.bool_value), 2: str(int(v.integer_value)), 3: repr(v.double_value),
                   4: str(v.string_value), 7: str(list(v.integer_array_value)),
                   8: str(list(v.double_array_value))}.get(v.type, f"<type {v.type}>")
            want = str(item["expect"])
            ok = got == want
            try:
                ok = abs(float(got) - float(want)) < 1e-6
            except ValueError:
                pass
            detail = f"{fqn} {name} = {got} (expect {want})"
            results.append(CheckResult("params", ok, detail + ("" if ok else "  <- MISMATCH (F4 class)")))
        except Exception as e:
            results.append(CheckResult("params", False, f"{fqn} {name}: node/param unavailable ({e})"))
    return results


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--profile", choices=["sim", "real"], default="sim")
    ap.add_argument("--profile-file", default=None, help="override profile yaml path")
    ap.add_argument("--only", default="tf,liveness,qos,params", help="comma list of check groups")
    ap.add_argument("--window", type=float, default=10.0, help="liveness window [s]")
    args = ap.parse_args()

    prof = load_profile(args)
    groups = {g.strip() for g in args.only.split(",") if g.strip()}

    rclpy.init()
    node = Node("system_smoke_node")
    from tf2_ros import Buffer, TransformListener
    node.tf_buffer = Buffer()
    node.tf_listener = TransformListener(node.tf_buffer, node)

    print(f"[smoke] profile={args.profile} window={args.window}s only={sorted(groups)} "
          f"domain={os.environ.get('ROS_DOMAIN_ID', '0')}")
    results: list = []

    if "tf" in groups:
        results += check_tf(node, prof)
    if "liveness" in groups:
        results += check_liveness(node, prof, args.window)
    if "qos" in groups:
        results += check_qos(prof)
    if "params" in groups:
        results += check_params(node, prof)

    n_pass = sum(1 for r in results if r.ok and not r.skipped)
    n_fail = sum(1 for r in results if not r.ok and not r.skipped)
    n_skip = sum(1 for r in results if r.skipped)
    for r in results:
        print(r.line())
    ok = n_fail == 0
    print(f"SUMMARY: {n_pass} PASS / {n_fail} FAIL / {n_skip} SKIP -> "
          f"{'SMOKE PASSED' if ok else 'SMOKE FAILED'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
