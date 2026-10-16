"""
Custom LED and Badge widgets for industrial status display.
"""

from PySide6.QtWidgets import QWidget, QHBoxLayout, QLabel, QFrame
from PySide6.QtGui import QPainter, QColor, QBrush, QPen
from PySide6.QtCore import Qt, QSize

from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_CYAN, COLOR_EMERALD,
    COLOR_EMERGENCY, COLOR_WARNING, COLOR_TEXT_PRIMARY, COLOR_TEXT_MUTED
)


class LEDWidget(QWidget):
    """
    Circular glowing indicator LED with multiple state colors.
    """
    COLORS = {
        "green": QColor(COLOR_EMERALD),
        "red": QColor(COLOR_EMERGENCY),
        "cyan": QColor(COLOR_CYAN),
        "amber": QColor(COLOR_WARNING),
        "gray": QColor(COLOR_TEXT_MUTED),
    }

    def __init__(self, color_name="gray", diameter=12, parent=None):
        super().__init__(parent)
        self._color_name = color_name
        self._diameter = diameter
        self.setFixedSize(diameter + 6, diameter + 6)

    def set_color(self, color_name: str):
        if color_name in self.COLORS and color_name != self._color_name:
            self._color_name = color_name
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        color = self.COLORS.get(self._color_name, self.COLORS["gray"])
        center = self.rect().center()
        radius = self._diameter / 2.0

        # Outer soft glow ring
        glow_color = QColor(color)
        glow_color.setAlpha(60)
        painter.setBrush(QBrush(glow_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(center, radius + 2.5, radius + 2.5)

        # Core LED
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(QColor(255, 255, 255, 120), 1))
        painter.drawEllipse(center, radius, radius)


class StatusBadge(QFrame):
    """
    Industrial badge with integrated LED, title, and state text.
    """
    def __init__(self, title: str, initial_text: str = "OFFLINE", initial_state: str = "red", parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                padding: 2px 6px;
            }}
        """)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 3, 6, 3)
        layout.setSpacing(6)

        self.led = LEDWidget(color_name=initial_state, diameter=10)
        layout.addWidget(self.led)

        self.lbl_title = QLabel(f"{title}:")
        self.lbl_title.setStyleSheet(f"color: {COLOR_TEXT_MUTED}; font-size: 10px; font-weight: 700;")
        layout.addWidget(self.lbl_title)

        self.lbl_value = QLabel(initial_text)
        self.lbl_value.setStyleSheet(f"color: {COLOR_TEXT_PRIMARY}; font-size: 11px; font-weight: 700;")
        layout.addWidget(self.lbl_value)

    def set_status(self, text: str, state_color: str):
        """Update badge text and LED state."""
        self.lbl_value.setText(text)
        self.led.set_color(state_color)
        if state_color == "green":
            self.lbl_value.setStyleSheet(f"color: {COLOR_EMERALD}; font-size: 11px; font-weight: 700;")
        elif state_color == "cyan":
            self.lbl_value.setStyleSheet(f"color: {COLOR_CYAN}; font-size: 11px; font-weight: 700;")
        elif state_color == "amber":
            self.lbl_value.setStyleSheet(f"color: {COLOR_WARNING}; font-size: 11px; font-weight: 700;")
        elif state_color == "red":
            self.lbl_value.setStyleSheet(f"color: {COLOR_EMERGENCY}; font-size: 11px; font-weight: 700;")
        else:
            self.lbl_value.setStyleSheet(f"color: {COLOR_TEXT_PRIMARY}; font-size: 11px; font-weight: 700;")
