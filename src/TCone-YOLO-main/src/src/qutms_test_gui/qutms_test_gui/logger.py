"""Unified logging system for qutms_test_gui.

Provides timestamp/level/module/message formatted logs, output to both
file and a Qt signal so the GUI Logs page can display them in real time.
"""

import logging
import os
from datetime import datetime

from PySide6.QtCore import QObject, Signal


class LogSignal(QObject):
    """Qt signal bridge so logging records reach the GUI thread."""

    log_emitted = Signal(str, str, str, str)  # timestamp, level, module, message


class GuiLogHandler(logging.Handler):
    """Logging handler that forwards records to a Qt signal."""

    def __init__(self, signal_bridge: LogSignal):
        super().__init__()
        self._signal = signal_bridge

    def emit(self, record: logging.LogRecord):
        try:
            msg = self.format(record)
            ts = datetime.fromtimestamp(record.created).strftime("%H:%M:%S")
            level = record.levelname
            module = record.name.split(".")[-1].upper()
            self._signal.log_emitted.emit(ts, level, module, msg)
        except Exception:
            pass


_log_signal = LogSignal()


def get_log_signal() -> LogSignal:
    return _log_signal


def setup_logger(name: str = "qutms_test_gui", log_dir: str = "logs") -> logging.Logger:
    """Configure and return the package logger.

    Logs go to:
      * console (stderr)
      * file at <log_dir>/qutms_gui.log
      * Qt signal consumed by the GUI Logs page
    """
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    logger.setLevel(logging.DEBUG)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    console.setFormatter(fmt)
    logger.addHandler(console)

    try:
        os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.FileHandler(os.path.join(log_dir, "qutms_gui.log"))
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(fmt)
        logger.addHandler(file_handler)
    except OSError:
        pass

    gui_handler = GuiLogHandler(_log_signal)
    gui_handler.setLevel(logging.DEBUG)
    gui_handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(gui_handler)

    return logger
