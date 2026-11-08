from PySide6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QMessageBox


class ExportTab(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.layout = QVBoxLayout()

        export_button = QPushButton("Export to .DBC File")
        export_button.clicked.connect(self.export_to_dbc)
        self.layout.addWidget(export_button)

        self.setLayout(self.layout)

    def export_to_dbc(self):
        if not self.parent.message_data:
            QMessageBox.warning(self, "Error", "No messages available to export.")
            return
        try:
            with open("output.dbc", "w") as f:
                f.write("VERSION \"\"\n\n")
                f.write("NS_ :\n\tNS_DESC_\n\tCM_\n\tBA_DEF_\n\tBA_\n\tVAL_\n\tCAT_DEF_\n\tCAT_\n\tFILTER\n\tBA_DEF_DEF_\n")
                f.write("\tEV_DATA_\n\tENVVAR_DATA_\n\tSGTYPE_\n\tSGTYPE_VAL_\n\tBA_REL_\n\tBA_DEF_REL_\n\tBA_DEF_DEF_REL_\n")
                f.write("\tBU_SG_REL_\n\tBU_EV_REL_\n\tBU_BO_REL_\n\tSG_MUL_VAL_\n\n")
                f.write("BS_:\n\n")
                f.write("BU_: {}\n\n".format(self.parent.ecu_data.get("name", "ECU")))

                for msg in self.parent.message_data:
                    size = msg.get('size', '8')
                    msg_line = f"BO_ {msg['id']} {msg['name']}: {size} {msg['source']}\n"
                    f.write(msg_line)

                    for sig in self.parent.signal_data.get(msg['id'], []):
                        start = sig.get('bit_start', '0')
                        length = sig.get('size', '8')
                        endianess = sig.get('endianess', 'Intel')
                        signed = sig.get('signed', 'Sem Sinal')

                        endian = '0' if 'Intel' in endianess else '1'
                        sign = '-' if 'Com Sinal' in signed else '+'

                        factor = sig.get('factor') or '1'
                        offset = sig.get('offset') or '0'
                        min_val = sig.get('min') or '0'
                        max_val = sig.get('max') or '0'
                        unit = sig.get('unit') or ''
                        values = ', '.join(sig.get('values', []))

                        signal_line = (
                            f" SG_ {sig['name']} : {start}|{length}@{endian}{sign} "
                            f"({factor},{offset}) [{min_val}|{max_val}] \"{unit}\" Vector__XXX\n"
                        )
                        f.write(signal_line)

                        if values:
                            comment_line = f"CM_ SG_ {msg['id']} {sig['name']} \"{values}\";\n"
                            f.write(comment_line)

            QMessageBox.information(self, "Success", "File 'output.dbc' exported successfully!")
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))
