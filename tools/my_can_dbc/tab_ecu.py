from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QComboBox, QCheckBox, QMessageBox


class EcuTab(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.layout = QVBoxLayout()

        self.ecu_name_input = QLineEdit()
        self.layout.addWidget(QLabel("ECU Name:"))
        self.layout.addWidget(self.ecu_name_input)

        self.ecu_address_input = QLineEdit()
        self.layout.addWidget(QLabel("Physical Address (0-253):"))
        self.layout.addWidget(self.ecu_address_input)

        self.ecu_manufacturer_input = QLineEdit()
        self.layout.addWidget(QLabel("Manufacturer:"))
        self.layout.addWidget(self.ecu_manufacturer_input)

        self.ecu_function_input = QLineEdit()
        self.layout.addWidget(QLabel("Function (e.g., Engine, Diagnostics):"))
        self.layout.addWidget(self.ecu_function_input)

        self.ecu_role_input = QComboBox()
        self.ecu_role_input.addItems(["Transmitter", "Receiver", "Transmitter/Receiver"])
        self.layout.addWidget(QLabel("ECU Role:"))
        self.layout.addWidget(self.ecu_role_input)

        self.ecu_fd_support = QCheckBox("Supports CAN FD")
        self.layout.addWidget(self.ecu_fd_support)

        save_button = QPushButton("Save ECU Properties")
        save_button.clicked.connect(self.save_ecu_info)
        self.layout.addWidget(save_button)

        self.setLayout(self.layout)

    def save_ecu_info(self):
        name = self.ecu_name_input.text()
        address = self.ecu_address_input.text()
        manufacturer = self.ecu_manufacturer_input.text()
        function = self.ecu_function_input.text()
        role = self.ecu_role_input.currentText()
        fd_support = self.ecu_fd_support.isChecked()

        if all([name, address, manufacturer, function]):
            self.parent.ecu_data = {
                'name': name,
                'address': address,
                'manufacturer': manufacturer,
                'function': function,
                'role': role,
                'fd_support': fd_support
            }
            QMessageBox.information(self, "Success", "ECU information saved successfully!")
        else:
            QMessageBox.warning(self, "Error", "Please fill in all required fields!")
