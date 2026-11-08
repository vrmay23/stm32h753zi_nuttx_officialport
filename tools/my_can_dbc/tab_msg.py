from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox, QScrollArea, QFrame, QMessageBox


class MessageTab(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.layout = QVBoxLayout()
        self.message_entries = []

        add_button = QPushButton("+ Add New Message")
        add_button.clicked.connect(self.add_message_entry)
        self.layout.addWidget(add_button)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.container = QWidget()
        self.container_layout = QVBoxLayout()
        self.container.setLayout(self.container_layout)
        self.scroll_area.setWidget(self.container)
        self.layout.addWidget(self.scroll_area)

        self.setLayout(self.layout)

    def add_message_entry(self):
        frame = QFrame()
        layout = QVBoxLayout()

        name_input = QLineEdit()
        id_input = QLineEdit()
        pgn_input = QLineEdit()
        prio_input = QLineEdit()
        interval_input = QLineEdit()
        size_input = QLineEdit()

        type_input = QComboBox()
        type_input.addItems(["Cyclic", "Event", "OnChange", "OnRequest"])

        protocol_input = QComboBox()
        protocol_input.addItems(["J1939-21", "J1939-73", "J1939-81"])

        category_input = QComboBox()
        category_input.addItems(["Diagnostics", "Application", "Network"])

        source_input = QLineEdit()
        destination_input = QLineEdit()

        layout.addWidget(QLabel("Message Name:"))
        layout.addWidget(name_input)

        layout.addWidget(QLabel("Message ID (Hex):"))
        layout.addWidget(id_input)

        layout.addWidget(QLabel("PGN:"))
        layout.addWidget(pgn_input)

        layout.addWidget(QLabel("Priority (0-7):"))
        layout.addWidget(prio_input)

        layout.addWidget(QLabel("Send Interval (ms):"))
        layout.addWidget(interval_input)

        layout.addWidget(QLabel("Size (bytes):"))
        layout.addWidget(size_input)

        layout.addWidget(QLabel("Transmission Type:"))
        layout.addWidget(type_input)

        layout.addWidget(QLabel("Protocol Type:"))
        layout.addWidget(protocol_input)

        layout.addWidget(QLabel("Message Category:"))
        layout.addWidget(category_input)

        layout.addWidget(QLabel("Source Address:"))
        layout.addWidget(source_input)

        layout.addWidget(QLabel("Destination Address:"))
        layout.addWidget(destination_input)

        save_button = QPushButton("Save Message")
        layout.addWidget(save_button)

        def save():
            try:
                mid = id_input.text().strip().lower().replace("0x", "")
                msg_id_int = int(mid, 16)
            except ValueError:
                QMessageBox.warning(self, "Error", "Invalid ID format.")
                return

            category = category_input.currentText()
            if category == "Diagnostics" and msg_id_int > 0x1FFFFFFF:
                QMessageBox.warning(self, "Error", "Diagnostics messages require a 29-bit ID.")
                return
            if category != "Diagnostics" and msg_id_int > 0x7FF:
                QMessageBox.warning(self, "Error", "Application/Network messages require ID ≤ 11 bits.")
                return

            if not name_input.text() or not id_input.text() or not source_input.text():
                QMessageBox.warning(self, "Error", "Please fill in all required fields!")
                return

            msg = {
                'name': name_input.text(),
                'id': mid.upper(),
                'pgn': pgn_input.text(),
                'priority': prio_input.text(),
                'interval': interval_input.text(),
                'size': size_input.text(),
                'type': type_input.currentText(),
                'protocol': protocol_input.currentText(),
                'category': category,
                'source': source_input.text(),
                'destination': destination_input.text()
            }

            self.parent.message_data.append(msg)
            self.parent.signal_data[mid.upper()] = []
            QMessageBox.information(self, "Saved", f"Message {msg['name']} saved successfully.")
            self.parent.update_message_selector()

        save_button.clicked.connect(save)
        frame.setLayout(layout)
        self.container_layout.addWidget(frame)
