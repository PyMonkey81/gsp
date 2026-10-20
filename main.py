"""
Main entry point for Gough-Stewart 6-DOF Platform Trajectory Tracking GUI.
Compatible with Windows 10/11 and Ubuntu Linux (X11/Wayland).
"""

import os
import sys
import traceback

# On Linux (Ubuntu Wayland), force XCB platform plugin so PyOpenGL (GLViewWidget) renders reliably
if sys.platform.startswith("linux"):
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont

from ui.main_window import MainWindow
from ui.styles import INDUSTRIAL_DARK_STYLESHEET


def global_exception_hook(exctype, value, tb):
    """Log unhandled exceptions cleanly to terminal."""
    traceback_details = "".join(traceback.format_exception(exctype, value, tb))
    sys.stderr.write(f"\n[FATAL ERROR] Excepción no controlada:\n{traceback_details}\n")


def main():
    sys.excepthook = global_exception_hook

    # Initialize Qt Application
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    app.setApplicationName("Gough-Stewart 6-DOF Platform Controller")
    app.setOrganizationName("Robotics Lab")

    # Set default modern UI font (Windows: Segoe UI | Ubuntu/Linux: Ubuntu / Sans)
    font_family = "Ubuntu" if sys.platform.startswith("linux") else "Segoe UI"
    default_font = QFont(font_family, 10)
    default_font.setStyleHint(QFont.StyleHint.SansSerif)
    app.setFont(default_font)

    # Apply Industrial Dark Mode Theme
    app.setStyleSheet(INDUSTRIAL_DARK_STYLESHEET)

    # Create and show Main Window
    window = MainWindow()
    window.show()

    # Start Event Loop
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
