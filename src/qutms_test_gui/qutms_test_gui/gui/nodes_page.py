"""Nodes page: live ROS2 node list with name, namespace, status."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QHeaderView


class NodesPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.lbl_count = QLabel("Nodes: 0")
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Node Name", "Namespace", "Status"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.lbl_count)
        layout.addWidget(self.table)

    def update_nodes(self, graph: dict):
        nodes = graph.get("nodes", [])
        self.lbl_count.setText(f"Nodes: {len(nodes)}")
        self.table.setRowCount(len(nodes))
        for row, node in enumerate(nodes):
            name = node.get("name", "")
            ns = node.get("namespace", "/")
            self.table.setItem(row, 0, QTableWidgetItem(name))
            self.table.setItem(row, 1, QTableWidgetItem(ns))
            status = QTableWidgetItem("RUNNING")
            status.setForeground(Qt.green)
            self.table.setItem(row, 2, status)
