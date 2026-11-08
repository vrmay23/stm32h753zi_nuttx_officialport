import sys
from PySide6.QtWidgets import QApplication
from gui import EditorDBC

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EditorDBC()
    window.show()
    sys.exit(app.exec())
