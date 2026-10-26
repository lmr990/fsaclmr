"""Process manager for launching and supervising eufs_sim / QUTMS launch.

Wraps QProcess so the GUI never blocks on subprocess I/O. Tracks PID,
runtime state, exit code, stdout and stderr, and emits Qt signals for
state transitions and output lines.
"""

import time
from dataclasses import dataclass
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
    """Manages a single long-lived ros2 launch subprocess."""

    state_changed = Signal(str)  # STARTING | RUNNING | STOPPED | ERROR
    stdout_line = Signal(str)
    stderr_line = Signal(str)
    # Payload-free: see ros_manager/vehicle_state for the PySide6 crash.
    process_info = Signal()

    def __init__(self):
        super().__init__()
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

    def start(self, program: str, args: list):
        """Launch a process. args is a list of strings."""
        if self.is_running:
            logger.warn("Process already running; stop it first")
            return

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