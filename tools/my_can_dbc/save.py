import json
from PySide6.QtWidgets import QFileDialog, QMessageBox


def save_project(parent):
    path, _ = QFileDialog.getSaveFileName(parent, "Save Project", filter="JSON Files (*.json)")
    if path:
        try:
            data = {
                "ecu": parent.ecu_data,
                "messages": parent.message_data,
                "signals": parent.signal_data
            }
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            QMessageBox.information(parent, "Success", "Project saved successfully!")
        except Exception as e:
            QMessageBox.critical(parent, "Save Error", str(e))


def load_project(parent):
    path, _ = QFileDialog.getOpenFileName(parent, "Load Project", filter="JSON Files (*.json)")
    if path:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
                parent.ecu_data = data.get("ecu", {})
                parent.message_data = data.get("messages", [])
                parent.signal_data = data.get("signals", {})

                # Restore ECU fields
                parent.tab_ecu.ecu_name_input.setText(parent.ecu_data.get("name", ""))
                parent.tab_ecu.ecu_address_input.setText(parent.ecu_data.get("address", ""))
                parent.tab_ecu.ecu_manufacturer_input.setText(parent.ecu_data.get("manufacturer", ""))
                parent.tab_ecu.ecu_function_input.setText(parent.ecu_data.get("function", ""))
                parent.tab_ecu.ecu_role_input.setCurrentText(parent.ecu_data.get("role", ""))
                parent.tab_ecu.ecu_fd_support.setChecked(parent.ecu_data.get("fd_support", False))

                # Refresh signal selector
                parent.update_message_selector()
                QMessageBox.information(parent, "Success", "Project loaded successfully!")
        except Exception as e:
            QMessageBox.critical(parent, "Load Error", str(e))
