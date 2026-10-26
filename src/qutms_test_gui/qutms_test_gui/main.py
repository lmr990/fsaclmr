"""Entry point for the QUTMS x EUFS test and control GUI."""

import sys

from PySide6.QtWidgets import QApplication

from qutms_test_gui.gui.main_window import MainWindow
from qutms_test_gui.logger import setup_logger

logger = setup_logger("qutms_test_gui")


def main():
    logger.info("Starting QUTMS x EUFS test GUI")
    app = QApplication(sys.argv)
    app.setApplicationName("QUTMS x EUFS Test GUI")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
