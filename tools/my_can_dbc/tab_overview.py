from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QTreeWidget, QTreeWidgetItem


class OverviewTab(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.layout = QVBoxLayout()

        self.tree = QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Element", "Details"])

        self.refresh_button = QPushButton("Refresh View")
        self.refresh_button.clicked.connect(self.update_view)

        self.layout.addWidget(QLabel("Project Overview"))
        self.layout.addWidget(self.refresh_button)
        self.layout.addWidget(self.tree)

        self.setLayout(self.layout)

    def update_view(self):
        self.tree.clear()

        ecu_name = self.parent.ecu_data.get('name', 'ECU')
        ecu_info = f"Address: {self.parent.ecu_data.get('address', '-')}, Function: {self.parent.ecu_data.get('function', '-')}"
        ecu_item = QTreeWidgetItem([f"ECU: {ecu_name}", ecu_info])
        self.tree.addTopLevelItem(ecu_item)

        for msg in self.parent.message_data:
            msg_id = msg['id']
            msg_label = f"{msg['name']} (ID: {msg_id})"
            msg_info = (
                f"PGN: {msg.get('pgn')} | Size: {msg.get('size')} | "
                f"Type: {msg['type']} | Priority: {msg.get('priority')} | Interval: {msg.get('interval')}ms"
            )
            msg_item = QTreeWidgetItem([msg_label, msg_info])
            ecu_item.addChild(msg_item)

            for sig in self.parent.signal_data.get(msg_id, []):
                sig_label = f"{sig['name']} ({sig['number']})"
                sig_detail = (
                    f"{sig['bit_start']}|{sig['size']} bits {sig.get('unit', '')} "
                    f"- Min: {sig.get('min', '')} Max: {sig.get('max', '')}"
                )
                sig_item = QTreeWidgetItem([sig_label, sig_detail])
                msg_item.addChild(sig_item)

        self.tree.expandAll()
