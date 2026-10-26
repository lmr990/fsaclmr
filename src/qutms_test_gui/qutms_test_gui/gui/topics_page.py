"""Topics page: live ROS2 topic list with message type, pub/sub counts."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QHeaderView


class TopicsPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.lbl_count = QLabel("Topics: 0")
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(
            ["Topic", "Message Type", "Publishers", "Subscribers", "Freq (Hz)"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.lbl_count)
        layout.addWidget(self.table)
        self._stats = {}

    def update_topics(self, graph: dict):
        topics = graph.get("topics", [])
        self.lbl_count.setText(f"Topics: {len(topics)}")
        self.table.setRowCount(len(topics))
        for row, topic in enumerate(topics):
            self.table.setItem(row, 0, QTableWidgetItem(topic["name"]))
            self.table.setItem(row, 1, QTableWidgetItem(", ".join(topic.get("types", []))))
            self.table.setItem(row, 2, QTableWidgetItem(str(topic.get("publishers", 0))))
            self.table.setItem(row, 3, QTableWidgetItem(str(topic.get("subscribers", 0))))
            rate = self._stats.get(topic["name"], {}).get("rate_hz", "-")
            self.table.setItem(row, 4, QTableWidgetItem(str(rate)))

    def update_topic_stats(self, stats: dict):
        self._stats = stats
