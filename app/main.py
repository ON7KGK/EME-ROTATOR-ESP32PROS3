"""
EME Rotator Controller - Application Windows
ON7KGK - 10 GHz EME
"""

import sys
from PySide6.QtWidgets import QApplication
from main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("EME Rotator Controller")
    app.setOrganizationName("ON7KGK")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
