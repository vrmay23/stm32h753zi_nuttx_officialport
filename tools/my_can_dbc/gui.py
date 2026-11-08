from PySide6.QtWidgets import QMainWindow, QTabWidget, QMenuBar, QMenu
from PySide6.QtGui import QAction

from tab_ecu import EcuTab
from tab_msg import MessageTab
from tab_signal import SignalTab
from tab_overview import OverviewTab
from tab_export import ExportTab
from save import save_project, load_project


class EditorDBC(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Professional DBC File Editor")
        self.setGeometry(100, 100, 1000, 600)

        # Core project data
        self.ecu_data = {}
        self.message_data = []
        self.signal_data = {}

        # Tabs
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        self.tab_ecu = EcuTab(self)
        self.tab_widget.addTab(self.tab_ecu, "ECU")

        self.tab_message = MessageTab(self)
        self.tab_widget.addTab(self.tab_message, "Messages")

        self.tab_signals = SignalTab(self)
        self.tab_widget.addTab(self.tab_signals, "Signals")

        self.tab_overview = OverviewTab(self)
        self.tab_widget.addTab(self.tab_overview, "Overview")

        self.tab_export = ExportTab(self)
        self.tab_widget.addTab(self.tab_export, "Export")

        self.setup_menu()

    def update_message_selector(self):
        self.tab_signals.update_selector()

    def setup_menu(self):
        menu_bar = QMenuBar(self)
        file_menu = QMenu("File", self)

        save_action = QAction("Save Project", self)
        save_action.triggered.connect(lambda: save_project(self))
        file_menu.addAction(save_action)

        load_action = QAction("Load Project", self)
        load_action.triggered.connect(lambda: load_project(self))
        file_menu.addAction(load_action)

        menu_bar.addMenu(file_menu)
        self.setMenuBar(menu_bar)
