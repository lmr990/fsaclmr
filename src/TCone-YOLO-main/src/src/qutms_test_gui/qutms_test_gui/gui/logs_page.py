"""Logs page: unified timestamp/level/module/message log display."""

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import QLabel, QPlainTextEdit, QVBoxLayout, QWidget


_LEVEL_COLORS = {
    "DEBUG": "gray",
    "INFO": "green",
    "WARN": "orange",
    "ERROR": "red",
}


class LogsPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.lbl_count = QLabel("Log entries: 0")
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMaximumBlockCount(5000)
        layout.addWidget(self.lbl_count)
        layout.addWidget(self.text)
        self._count = 0

    def append_log(self, timestamp: str, level: str, module: str, message: str):
        line = f"[{timestamp}] {level:<5} {module:<10} {message}"
        color = _LEVEL_COLORS.get(level, "black")
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor = self.text.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(line + "\n", fmt)
        self.text.setTextCursor(cursor)
        self.text.ensureCursorVisible()
        self._count += 1
        self.lbl_count.setText(f"Log entries: {self._count}")
