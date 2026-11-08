from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
                               QPushButton, QComboBox, QMessageBox)


class SignalTab(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.layout = QVBoxLayout()

        self.signal_msg_selector = QComboBox()
        self.layout.addWidget(QLabel("Select Message:"))
        self.layout.addWidget(self.signal_msg_selector)

        self.signal_container_layout = QVBoxLayout()

        add_button = QPushButton("+ Add Signal")
        add_button.clicked.connect(self.add_signal_entry)
        self.layout.addWidget(add_button)
        self.layout.addLayout(self.signal_container_layout)

        self.setLayout(self.layout)

    def update_selector(self):
        self.signal_msg_selector.clear()
        for msg in self.parent.message_data:
            self.signal_msg_selector.addItem(f"{msg['id']} - {msg['name']}")

    def add_signal_entry(self):
        container = QWidget()
        layout = QVBoxLayout()

        line1 = QHBoxLayout()
        signal_num_input = QLineEdit()
        signal_num_input.setPlaceholderText("00")
        signal_num_input.setMaxLength(2)
        name_input = QLineEdit()

        line1.addWidget(QLabel("Number (Hex):"))
        line1.addWidget(signal_num_input)
        line1.addWidget(QLabel("Name:"))
        line1.addWidget(name_input)
        layout.addLayout(line1)

        line2 = QHBoxLayout()
        start_input = QLineEdit()
        size_input = QLineEdit()
        range_input = QLineEdit()

        line2.addWidget(QLabel("Start Position:"))
        line2.addWidget(start_input)
        line2.addWidget(QLabel("Length (bits):"))
        line2.addWidget(size_input)
        line2.addWidget(QLabel("Range:"))
        line2.addWidget(range_input)
        layout.addLayout(line2)

        line3 = QHBoxLayout()
        endian_input = QComboBox()
        endian_input.addItems(["Intel (Little Endian)", "Motorola (Big Endian)"])
        signed_input = QComboBox()
        signed_input.addItems(["Unsigned", "Signed"])
        unit_input = QLineEdit()

        line3.addWidget(QLabel("Endianess:"))
        line3.addWidget(endian_input)
        line3.addWidget(QLabel("Signed:"))
        line3.addWidget(signed_input)
        line3.addWidget(QLabel("Unit:"))
        line3.addWidget(unit_input)
        layout.addLayout(line3)

        line4 = QHBoxLayout()
        factor_input = QLineEdit()
        offset_input = QLineEdit()

        line4.addWidget(QLabel("Factor:"))
        line4.addWidget(factor_input)
        line4.addWidget(QLabel("Offset:"))
        line4.addWidget(offset_input)
        layout.addLayout(line4)

        line5 = QHBoxLayout()
        min_input = QLineEdit()
        max_input = QLineEdit()

        line5.addWidget(QLabel("Min:"))
        line5.addWidget(min_input)
        line5.addWidget(QLabel("Max:"))
        line5.addWidget(max_input)
        layout.addLayout(line5)

        values_layout = QVBoxLayout()
        layout.addWidget(QLabel("Enumerated Values (if any):"))
        layout.addLayout(values_layout)
        value_inputs = []

        def update_values():
            for i in reversed(range(values_layout.count())):
                widget = values_layout.itemAt(i).widget()
                if widget:
                    widget.setParent(None)
            value_inputs.clear()
            try:
                qtd = int(range_input.text())
                for i in range(qtd):
                    val_input = QLineEdit()
                    val_input.setPlaceholderText(f"Value{i}")
                    values_layout.addWidget(val_input)
                    value_inputs.append(val_input)
            except ValueError:
                pass

        range_input.textChanged.connect(update_values)

        save_button = QPushButton("Save Signal")

        def save():
            msg_key = self.signal_msg_selector.currentText().split(" - ")[0]
            if not msg_key or msg_key not in self.parent.signal_data:
                QMessageBox.warning(self, "Error", "Selected message not found.")
                return
            if name_input.text() and start_input.text() and size_input.text():
                signal = {
                    'number': signal_num_input.text(),
                    'name': name_input.text(),
                    'bit_start': start_input.text(),
                    'size': size_input.text(),
                    'range': range_input.text(),
                    'endianess': endian_input.currentText(),
                    'signed': signed_input.currentText(),
                    'unit': unit_input.text(),
                    'factor': factor_input.text(),
                    'offset': offset_input.text(),
                    'min': min_input.text(),
                    'max': max_input.text(),
                    'values': [v.text() for v in value_inputs]
                }
                self.parent.signal_data[msg_key].append(signal)
                QMessageBox.information(self, "Success", f"Signal {name_input.text()} saved successfully.")
            else:
                QMessageBox.warning(self, "Error", "Please fill in all required fields!")

        layout.addWidget(save_button)
        save_button.clicked.connect(save)

        container.setLayout(layout)
        self.signal_container_layout.addWidget(container)
