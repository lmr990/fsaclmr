"""Regression test for the stale-GUI-PYTHONPATH crash.

The GUI inherits PYTHONPATH when it launches. A package built afterwards
(e.g. yolo_cone_detection) has no entry there, and child nodes die with
PackageNotFoundError because importlib.metadata does not follow egg-links.
ProcessManager._refresh_workspace_pythonpath re-scans install/ on every
process start so children always see the current overlay.
"""

import os
from pathlib import Path

import pytest

from qutms_test_gui.process.process_manager import ProcessManager

OPT_ROS_PATHS = [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
]


def _yolo_paths() -> "list[str]":
    """yolo's build dir and install site-packages, as the refresh maps them."""
    ws = ProcessManager._workspace_root()
    alias = ProcessManager._space_free_alias(ws) if " " in ws else ws
    yolo_dirs = list(Path(ws).glob("install/yolo_cone_detection/lib/python3.*/site-packages"))
    assert yolo_dirs, "yolo_cone_detection must be built for this test"
    paths = [str(yolo_dirs[0]), str(Path(ws) / "build" / "yolo_cone_detection")]
    return [p.replace(ws, alias) for p in paths]


def _stale_env() -> "object":
    """PYTHONPATH as the GUI would have it if started before yolo was built.

    Built explicitly (all known workspace paths except yolo's, then /opt/ros)
    instead of from os.environ: systemEnvironment() couples the test to the
    shell it runs in and to env mutations from earlier tests.
    """
    from PySide6.QtCore import QProcessEnvironment

    ws = ProcessManager._workspace_root()
    alias = ProcessManager._space_free_alias(ws) if " " in ws else ws
    prefix = alias if alias is not None else ws

    stale = list(OPT_ROS_PATHS)
    for pkg in sorted((Path(ws) / "install").iterdir()):
        if pkg.name == "yolo_cone_detection":
            continue
        for sub in ("lib/python3.*/site-packages",
                    "lib/python3.*/dist-packages",
                    "local/lib/python3.*/dist-packages"):
            stale += [str(p).replace(ws, prefix) for p in pkg.glob(sub)]
        build_dir = Path(ws) / "build" / pkg.name
        if (build_dir / "setup.py").is_file():
            stale.append(str(build_dir).replace(ws, prefix))

    env = QProcessEnvironment()
    env.insert("PYTHONPATH", os.pathsep.join(stale))
    return env


class TestRefreshWorkspacePythonpath:
    def test_missing_package_paths_are_added(self):
        pm = ProcessManager()
        env = _stale_env()
        assert "yolo_cone_detection" not in env.value("PYTHONPATH")

        pm._refresh_workspace_pythonpath(env)

        pp = env.value("PYTHONPATH").split(os.pathsep)
        build_dir, yolo = _yolo_paths()
        assert yolo in pp
        # importlib.metadata reads the egg-info in build/, so a fresh
        # symlink-install layout (egg-link only on the install side) needs
        # the build dir on PYTHONPATH too.
        assert build_dir in pp

    def test_workspace_paths_shadow_underlay(self):
        pm = ProcessManager()
        env = _stale_env()
        pm._refresh_workspace_pythonpath(env)

        pp = env.value("PYTHONPATH").split(os.pathsep)
        build_dir, yolo = _yolo_paths()
        first_underlay = min(
            (i for i, p in enumerate(pp) if p.startswith("/opt/ros")),
            default=len(pp),
        )
        assert pp.index(yolo) < first_underlay
        assert pp.index(build_dir) < first_underlay

    def test_refresh_is_idempotent(self):
        pm = ProcessManager()
        env = _stale_env()
        pm._refresh_workspace_pythonpath(env)
        once = env.value("PYTHONPATH")
        pm._refresh_workspace_pythonpath(env)
        assert env.value("PYTHONPATH") == once


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
