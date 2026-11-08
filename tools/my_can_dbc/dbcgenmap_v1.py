import os
from PySide6.QtWidgets import (QApplication, QMainWindow, QTabWidget, QWidget, QVBoxLayout, QLabel, QLineEdit,
                               QPushButton, QMessageBox, QComboBox, QHBoxLayout, QScrollArea, QFrame,
                               QTreeWidget, QTreeWidgetItem)


class EditorDBC(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Editor Gráfico de Arquivos DBC Profissional")
        self.setGeometry(100, 100, 1000, 600)

        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        self.ecu_data = {}
        self.message_data = []
        self.signal_data = {}

        self.add_ecu_tab()
        self.add_message_tab()
        self.add_signal_tab()
        self.add_export_tab()
        self.add_summary_tab()

    def add_ecu_tab(self):
        ecu_tab = QWidget()
        layout = QVBoxLayout()

        self.ecu_name_input = QLineEdit()
        layout.addWidget(QLabel("Nome da ECU:"))
        layout.addWidget(self.ecu_name_input)

        self.ecu_address_input = QLineEdit()
        layout.addWidget(QLabel("Endereço da ECU:"))
        layout.addWidget(self.ecu_address_input)

        save_button = QPushButton("Salvar Características da ECU")
        save_button.clicked.connect(self.save_ecu_info)
        layout.addWidget(save_button)

        ecu_tab.setLayout(layout)
        self.tab_widget.addTab(ecu_tab, "ECU")

    def save_ecu_info(self):
        name = self.ecu_name_input.text()
        address = self.ecu_address_input.text()
        if name and address:
            self.ecu_data = {'name': name, 'address': address}
            QMessageBox.information(self, "Sucesso", "Informações da ECU salvas!")
        else:
            QMessageBox.warning(self, "Erro", "Preencha todos os campos!")

    def add_message_tab(self):
        self.message_tab = QWidget()
        self.message_layout = QVBoxLayout()
        self.message_entries = []

        add_button = QPushButton("+ Adicionar Nova Mensagem")
        add_button.clicked.connect(self.add_message_entry)
        self.message_layout.addWidget(add_button)

        self.message_scroll = QScrollArea()
        self.message_scroll.setWidgetResizable(True)

        self.message_container = QWidget()
        self.message_container_layout = QVBoxLayout()
        self.message_container.setLayout(self.message_container_layout)

        self.message_scroll.setWidget(self.message_container)
        self.message_layout.addWidget(self.message_scroll)

        self.message_tab.setLayout(self.message_layout)
        self.tab_widget.addTab(self.message_tab, "Mensagens")

    def add_message_entry(self):
        frame = QFrame()
        layout = QVBoxLayout()

        name_input = QLineEdit()
        id_input = QLineEdit()
        type_input = QComboBox()
        type_input.addItems(["Cyclic", "Event", "OnChange"])
        source_input = QLineEdit()
        destination_input = QLineEdit()
        category_input = QComboBox()
        category_input.addItems(["Diagnóstico", "Aplicação", "Rede"])

        layout.addWidget(QLabel("Nome da Mensagem:"))
        layout.addWidget(name_input)
        layout.addWidget(QLabel("ID da Mensagem (Hex):"))
        layout.addWidget(id_input)
        layout.addWidget(QLabel("Tipo de Envio:"))
        layout.addWidget(type_input)
        layout.addWidget(QLabel("Categoria da Mensagem:"))
        layout.addWidget(category_input)
        layout.addWidget(QLabel("Source Address:"))
        layout.addWidget(source_input)
        layout.addWidget(QLabel("Destination Address:"))
        layout.addWidget(destination_input)

        save_button = QPushButton("Salvar Mensagem")
        layout.addWidget(save_button)

        def save():
            name = name_input.text()
            mid = id_input.text().strip().lower().replace("0x", "")
            mtype = type_input.currentText()
            category = category_input.currentText()
            source = source_input.text()
            dest = destination_input.text()

            try:
                msg_id_int = int(mid, 16)
            except ValueError:
                QMessageBox.warning(self, "Erro", "ID deve ser um número hexadecimal válido.")
                return

            if category == "Diagnóstico" and msg_id_int > 0x1FFFFFFF:
                QMessageBox.warning(self, "Erro", "Mensagens de diagnóstico devem ter até 29 bits (ID ≤ 0x1FFFFFFF).")
                return
            if category != "Diagnóstico" and msg_id_int > 0x7FF:
                QMessageBox.warning(self, "Erro", "Mensagens de aplicação ou rede devem ter ID de até 11 bits (ID ≤ 0x7FF).")
                return

            if name and mid and source and dest:
                msg = {
                    'name': name,
                    'id': mid.upper(),
                    'type': mtype,
                    'category': category,
                    'source': source,
                    'destination': dest
                }
                self.message_data.append(msg)
                self.signal_data[mid.upper()] = []
                QMessageBox.information(self, "Salvo", f"Mensagem {name} salva.")
                self.update_message_selector()
            else:
                QMessageBox.warning(self, "Erro", "Preencha todos os campos!")

        save_button.clicked.connect(save)
        frame.setLayout(layout)
        self.message_container_layout.addWidget(frame)
        self.message_entries.append((name_input, id_input, type_input, category_input, source_input, destination_input))

    def add_signal_tab(self):
        self.signal_tab = QWidget()
        self.signal_layout = QVBoxLayout()

        self.signal_msg_selector = QComboBox()
        self.signal_layout.addWidget(QLabel("Selecionar Mensagem:"))
        self.signal_layout.addWidget(self.signal_msg_selector)

        self.signal_container_layout = QVBoxLayout()

        add_signal_button = QPushButton("+ Adicionar Sinal")
        add_signal_button.clicked.connect(self.add_signal_entry)
        self.signal_layout.addWidget(add_signal_button)
        self.signal_layout.addLayout(self.signal_container_layout)

        self.signal_tab.setLayout(self.signal_layout)
        self.tab_widget.addTab(self.signal_tab, "Sinais")

    def update_message_selector(self):
        self.signal_msg_selector.clear()
        for msg in self.message_data:
            self.signal_msg_selector.addItem(f"{msg['id']} - {msg['name']}")

    def add_signal_entry(self):
        container = QWidget()
        layout = QVBoxLayout()

        linha1 = QHBoxLayout()
        signal_num_input = QLineEdit()
        signal_num_input.setPlaceholderText("00")
        signal_num_input.setMaxLength(2)
        linha1.addWidget(QLabel("Número (Hex):"))
        linha1.addWidget(signal_num_input)

        name_input = QLineEdit()
        linha1.addWidget(QLabel("Nome:"))
        linha1.addWidget(name_input)

        layout.addLayout(linha1)

        linha2 = QHBoxLayout()
        start_input = QLineEdit()
        length_input = QLineEdit()
        range_input = QLineEdit()

        linha2.addWidget(QLabel("Start_Position:"))
        linha2.addWidget(start_input)
        linha2.addWidget(QLabel("Length (bits):"))
        linha2.addWidget(length_input)
        linha2.addWidget(QLabel("Range:"))
        linha2.addWidget(range_input)

        layout.addLayout(linha2)

        values_layout = QVBoxLayout()
        layout.addWidget(QLabel("Valores possíveis:"))
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

        save_button = QPushButton("Salvar Sinal")

        def save():
            current_text = self.signal_msg_selector.currentText()
            if not current_text:
                QMessageBox.warning(self, "Erro", "Selecione uma mensagem.")
                return
            msg_key = current_text.split(" - ")[0]
            if msg_key not in self.signal_data:
                QMessageBox.warning(self, "Erro", "Mensagem não encontrada.")
                return

            if name_input.text() and start_input.text() and length_input.text() and range_input.text():
                signal = {
                    'number': signal_num_input.text(),
                    'name': name_input.text(),
                    'bit_start': start_input.text(),
                    'size': length_input.text(),
                    'range': range_input.text(),
                    'values': [v.text() for v in value_inputs]
                }
                self.signal_data[msg_key].append(signal)
                QMessageBox.information(self, "Salvo", f"Sinal {name_input.text()} salvo.")
            else:
                QMessageBox.warning(self, "Erro", "Preencha todos os campos obrigatórios!")

        layout.addWidget(save_button)
        save_button.clicked.connect(save)

        container.setLayout(layout)
        self.signal_container_layout.addWidget(container)

    def add_export_tab(self):
        export_tab = QWidget()
        layout = QVBoxLayout()

        export_button = QPushButton("Exportar .dbc")
        export_button.clicked.connect(self.export_to_dbc)
        layout.addWidget(export_button)

        export_tab.setLayout(layout)
        self.tab_widget.addTab(export_tab, "Exportação")

    def export_to_dbc(self):
        if not self.message_data:
            QMessageBox.warning(self, "Erro", "Nenhuma mensagem para exportar.")
            return
        try:
            with open("output.dbc", "w") as f:
                f.write("VERSION \"\"\n\n")
                f.write("BU_: ")
                if self.ecu_data.get("name"):
                    f.write(f"{self.ecu_data['name']}\n")
                else:
                    f.write("ECU\n")

                for msg in self.message_data:
                    f.write(f"\nBO_ {msg['id']} {msg['name']}: {msg['source']} ECU // {msg['category']}\n")
                    for sig in self.signal_data.get(msg['id'], []):
                        comment = ', '.join(sig.get('values', []))
                        f.write(
                            f" SG_ {sig['name']} : {sig['bit_start']}|{sig['size']}@1+ (1,0) [0|0] \"{comment}\" Vector__XXX\n"
                        )
            QMessageBox.information(self, "Sucesso", "Arquivo DBC exportado.")
        except Exception as e:
            QMessageBox.critical(self, "Erro", str(e))

    def add_summary_tab(self):
        self.summary_tab = QWidget()
        layout = QVBoxLayout()

        self.tree = QTreeWidget()
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Elemento", "Detalhes"])

        update_button = QPushButton("Atualizar Visualização")
        update_button.clicked.connect(self.update_tree)

        layout.addWidget(update_button)
        layout.addWidget(self.tree)

        self.summary_tab.setLayout(layout)
        self.tab_widget.addTab(self.summary_tab, "Visão Geral")

    def update_tree(self):
        self.tree.clear()
        for msg in self.message_data:
            msg_id = msg['id']
            msg_name = msg['name']
            category = msg['category']
            mtype = msg['type']
            msg_info = f"ID: {msg_id} | Tipo: {mtype} | Categoria: {category}"

            msg_item = QTreeWidgetItem([f"Mensagem: {msg_name}", msg_info])
            self.tree.addTopLevelItem(msg_item)

            for sig in self.signal_data.get(msg_id, []):
                sig_info = f"{sig['bit_start']}|{sig['size']} bits [{sig['range']}]"
                values = ', '.join(sig.get('values', []))
                sig_item = QTreeWidgetItem([f"Sinal {sig['number']}: {sig['name']}", f"{sig_info} -> {values}"])
                msg_item.addChild(sig_item)

        self.tree.expandAll()


if __name__ == "__main__":
    app = QApplication([])
    window = EditorDBC()
    window.show()
    app.exec()
