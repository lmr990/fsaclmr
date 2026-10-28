"""Process manager for launching and supervising eufs_sim / QUTMS launch.

Wraps QProcess so the GUI never blocks on subprocess I/O. Tracks PID,
runtime state, exit code, stdout and stderr, and emits Qt signals for
state transitions and output lines.
"""

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Signal

from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui.process_manager")


@dataclass
class ProcessInfo:
    pid: Optional[int] = None
    start_time: float = 0.0
    running: bool = False
    exit_code: Optional[int] = None
    exit_status: Optional[str] = None


class ProcessManager(QObject):
    """Manages a single long-lived ros2 launch subprocess.

    ``sweep_targets`` selects which leftover processes are terminated before
    every start (by process name or by basename of any command-line element).
    Sim managers use the Gazebo/rqt defaults; a manager for a pure-observer
    node passes its own target so starting it never kills a running
    simulator."""

    DEFAULT_SWEEP_TARGETS = frozenset({"gzserver", "gzclient", "rqt_gui"})

    state_changed = Signal(str)  # STARTING | RUNNING | STOPPED | ERROR
    stdout_line = Signal(str)
    stderr_line = Signal(str)
    # Payload-free: see ros_manager/vehicle_state for the PySide6 crash.
    process_info = Signal()

    def __init__(self, sweep_targets=None):
        super().__init__()
        self._sweep_targets = (
            frozenset(sweep_targets)
            if sweep_targets is not None
            else self.DEFAULT_SWEEP_TARGETS
        )
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.SeparateChannels)
        self._process.readyReadStandardOutput.connect(self._on_stdout)
        self._process.readyReadStandardError.connect(self._on_stderr)
        self._process.started.connect(self._on_started)
        self._process.finished.connect(self._on_finished)
        self._process.errorOccurred.connect(self._on_error)

        self._info = ProcessInfo()
        self._cmd = ""
        self._args = []

    @property
    def info(self) -> ProcessInfo:
        return self._info

    @property
    def is_running(self) -> bool:
        return self._process.state() == QProcess.Running

    # Env vars whose workspace-path entries must point at the space-free alias.
    _PREFIX_ENV_VARS = (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "PYTHONPATH",
        "LD_LIBRARY_PATH",
        "GAZEBO_MODEL_PATH",
        "GAZEBO_RESOURCE_PATH",
        "GAZEBO_PLUGIN_PATH",
        "GAZEBO_SYSTEM_PLUGIN_PATH",
        "ROS_PACKAGE_PATH",
    )

    @staticmethod
    def _workspace_root() -> Optional[str]:
        """Locate the workspace that owns this package (the dir with src/ and install/)."""
        for parent in Path(__file__).resolve().parents:
            if (parent / "install").is_dir() and (parent / "src").is_dir():
                return str(parent)
        return None

    @staticmethod
    def _space_free_alias(ws: str) -> Optional[str]:
        """Return a space-free symlink alias of ws, creating it if needed.

        resource_retriever (used by RViz and Gazebo for ``package://`` and
        ``file://`` mesh URIs) cannot open a path containing a space such as
        ``/home/lmr/qutms_ws (1)``: every mesh load fails. Returns None when
        the alias cannot be created or points elsewhere.
        """
        alias = ws.replace(" ", "_").replace("(", "").replace(")", "")
        if os.path.islink(alias):
            if os.path.realpath(alias) != os.path.realpath(ws):
                logger.warning("Space-free alias %s points elsewhere; not used", alias)
                return None
        elif os.path.exists(alias):
            logger.warning("Cannot create space-free alias %s: path exists", alias)
            return None
        else:
            try:
                os.symlink(ws, alias)
            except OSError as exc:
                logger.warning("Cannot create space-free alias %s: %s", alias, exc)
                return None
        return alias

    def _remap_space_free_prefix(self, env: QProcessEnvironment) -> None:
        """Point child package prefixes at a space-free alias of the workspace.

        In RViz a space in the workspace path hides the car chassis and floods
        the log with thousands of errors per second, starving the CPU so RViz
        and YOLO crawl. Child processes get their package prefixes rewritten
        to a symlink alias without spaces, e.g. ``/home/lmr/qutms_ws_1``.
        """
        ws = self._workspace_root()
        if ws is None or " " not in ws:
            return
        alias = self._space_free_alias(ws)
        if alias is None:
            return

        for var in self._PREFIX_ENV_VARS:
            value = env.value(var)
            if value and ws in value:
                env.insert(var, value.replace(ws, alias))
        logger.info("Using space-free workspace prefix %s for child processes", alias)

    def _refresh_workspace_pythonpath(self, env: QProcessEnvironment) -> None:
        """Add workspace python paths that appeared after the GUI started.

        The GUI inherits PYTHONPATH when it launches. If a package is built
        afterwards, its entries are missing from that inherited value and
        child nodes crash with PackageNotFoundError. Re-scan the workspace on
        every launch so children always see the current overlay; the user no
        longer needs to restart the GUI after rebuilding.

        For each package this mirrors what colcon's generated setup.sh does:
        both ``build/<pkg>`` (for ament_python packages — the egg-info that
        importlib.metadata reads lives there, while the install side only
        carries an egg-link it does not follow) and the
        ``install/<pkg>/lib/python3.*/`` site/dist-packages directories.
        """
        ws = self._workspace_root()
        if ws is None:
            return
        install = Path(ws) / "install"
        if not install.is_dir():
            return

        alias = self._space_free_alias(ws) if " " in ws else None
        prefix = alias if alias is not None else ws

        current = env.value("PYTHONPATH") or ""
        entries = [p for p in current.split(os.pathsep) if p]
        known = set(entries)
        missing = []
        for pkg_dir in sorted(install.iterdir()):
            pkg_paths = []
            build_dir = Path(ws) / "build" / pkg_dir.name
            if (build_dir / "setup.py").is_file():
                pkg_paths.append(str(build_dir))
            for sub in ("lib/python3.*/site-packages",
                        "lib/python3.*/dist-packages",
                        "local/lib/python3.*/dist-packages"):
                for pydir in pkg_dir.glob(sub):
                    pkg_paths.append(str(pydir))
            for path in pkg_paths:
                path = path.replace(ws, prefix)
                if path not in known:
                    known.add(path)
                    missing.append((pkg_dir.name, path))
        if not missing:
            return
        env.insert("PYTHONPATH", os.pathsep.join([p for _, p in missing] + entries))
        logger.info(
            "Refreshed PYTHONPATH with %d workspace path(s): %s",
            len(missing),
            ", ".join(pkg for pkg, _ in missing),
        )

    @staticmethod
    def _matches_sweep_target(info: dict, targets: set) -> bool:
        """Name match, or exact basename match on any command-line element.

        The basename pass catches entry-point scripts whose process name is
        truncated (e.g. ``path_tracker_node``) or launched via python3."""
        if (info.get("name") or "") in targets:
            return True
        for element in info.get("cmdline") or []:
            if os.path.basename(element) in targets:
                return True
        return False

    def _kill_stale_simulation(self):
        """Terminate Gazebo left behind by a previous simulation.

        ``ros2 launch`` does not reliably terminate its children when the GUI
        is closed or when the launch process is interrupted, so a gzserver can
        survive from an earlier session (possibly from another workspace).
        The next launch then starts a second gzserver that dies on the port
        conflict (exit code 255), while ``spawn_entity`` silently connects to
        the surviving server: the old world stays on screen and the car is
        spawned at the *new* track's CSV coordinates inside that old world,
        which looks like "the same track with wrong positions".
        """
        try:
            import psutil
        except ImportError:
            return

        targets = set(self._sweep_targets)
        if not targets:
            return
        stale = [
            proc
            for proc in psutil.process_iter(["name", "cmdline"])
            if self._matches_sweep_target(proc.info, targets)
        ]
        if not stale:
            return

        logger.warn(
            "Found stale simulation processes, terminating before launch: %s",
            ", ".join(f"{p.info['name']}(PID {p.pid})" for p in stale),
        )
        for proc in stale:
            try:
                proc.terminate()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        _gone, alive = psutil.wait_procs(stale, timeout=3)
        for proc in alive:
            try:
                logger.warn("Force-killing stale PID %s", proc.pid)
                proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def start(self, program: str, args: list):
        """Launch a process. args is a list of strings."""
        if self.is_running:
            logger.warn("Process already running; stop it first")
            return

        self._kill_stale_simulation()

        self._cmd = program
        self._args = args
        self._info = ProcessInfo(start_time=time.time(), running=False)
        self.state_changed.emit("STARTING")

        # Clean snap-injected GTK/GIO env vars so child processes (rviz2,
        # rqt_gui, gazebo) don't pull in snap's core20 libpthread which is
        # incompatible with the system glibc.
        env = QProcessEnvironment.systemEnvironment()
        for var in ("GTK_PATH", "GIO_MODULE_DIR", "GSETTINGS_SCHEMA_DIR",
                    "GTK_IM_MODULE_FILE", "LOCPATH"):
            env.remove(var)
        # Strip Python-package Qt plugin paths. Importing pip opencv (cv2)
        # inside this GUI process sets QT_QPA_PLATFORM_PLUGIN_PATH to its
        # own Qt6 plugins; a child rviz2 (system Qt5) then tries to load
        # those Qt6 plugins and aborts (exit -6, "Could not load the Qt
        # platform plugin xcb"). System Qt apps must use their own plugins.
        for var in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH",
                    "QML2_IMPORT_PATH", "QML_IMPORT_PATH"):
            env.remove(var)
        self._remap_space_free_prefix(env)
        self._refresh_workspace_pythonpath(env)
        self._process.setProcessEnvironment(env)

        logger.info("Starting process: %s %s", program, " ".join(args))
        self._process.start(program, args)

    def stop(self):
        """Stop the process and kill its entire process tree.

        ``ros2 launch`` spawns many child processes (gzserver, gzclient,
        rviz2, robot_state_publisher, ...) that do not terminate when the
        parent launch process is terminated. We therefore walk the process
        tree with psutil and terminate every descendant, then SIGKILL any
        survivors.
        """
        if not self.is_running:
            logger.info("Stop requested but process not running")
            return
        pid = self._info.pid
        logger.info("Stopping process tree rooted at PID %s", pid)
        try:
            import psutil

            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            procs = children + [parent]

            for p in procs:
                try:
                    p.terminate()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            gone, alive = psutil.wait_procs(procs, timeout=5)
            for p in alive:
                try:
                    logger.warn("Force-killing PID %s", p.pid)
                    p.kill()
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
        except ImportError:
            logger.warn("psutil not available; falling back to QProcess.terminate()")
            self._process.terminate()
            if not self._process.waitForFinished(5000):
                self._process.kill()
        except psutil.NoSuchProcess:
            logger.info("Process %s already gone", pid)

        # Safety net: ensure the QProcess object itself reports finished.
        if self._process.state() != QProcess.NotRunning:
            if not self._process.waitForFinished(2000):
                self._process.kill()
                self._process.waitForFinished(2000)

    def shutdown(self):
        """Stop the launch process and remove any surviving simulator child.

        ``stop()`` is a no-op when the launch has already exited, but Gazebo
        can still be alive in that case; on GUI close we always sweep it.
        """
        self.stop()
        self._kill_stale_simulation()

    def restart(self):
        self.stop()
        # QProcess emits finished after termination; restart there via a flag.
        self._pending_restart = True

    def _on_started(self):
        self._info.pid = self._process.processId()
        self._info.running = True
        self._info.exit_code = None
        self.state_changed.emit("RUNNING")
        self._emit_info()
        logger.info("Process started (PID %s)", self._info.pid)

    def _on_finished(self, exit_code, exit_status):
        self._info.running = False
        self._info.exit_code = exit_code
        self._info.exit_status = (
            "NORMAL" if exit_status == QProcess.NormalExit else "CRASHED"
        )
        self.state_changed.emit("STOPPED")
        self._emit_info()
        logger.info(
            "Process finished (exit_code=%s, status=%s)",
            exit_code,
            self._info.exit_status,
        )
        if getattr(self, "_pending_restart", False):
            self._pending_restart = False
            self.start(self._cmd, self._args)

    def _on_error(self, error):
        logger.error("Process error: %s", error)
        self.state_changed.emit("ERROR")

    def _on_stdout(self):
        data = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        for line in data.splitlines():
            if line:
                self.stdout_line.emit(line)

    def _on_stderr(self):
        data = bytes(self._process.readAllStandardError()).decode("utf-8", errors="replace")
        for line in data.splitlines():
            if line:
                self.stderr_line.emit(line)

    def _emit_info(self):
        self.process_info.emit()