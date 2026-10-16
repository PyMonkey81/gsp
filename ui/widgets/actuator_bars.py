"""
Actuator level display widgets for 6-DOF Gough-Stewart Platform.
Redesigned with compact horizontal header rows and live limit tracking.
"""

from typing import List, Union
import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QFrame, QGridLayout
)
from PySide6.QtCore import Qt

from config.parameters import Q_MIN, Q_MAX, Q_NOMINAL, Q_SAFE_MIN, Q_SAFE_MAX
from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_CYAN, COLOR_EMERALD,
    COLOR_EMERGENCY, COLOR_WARNING, COLOR_TEXT_PRIMARY,
    COLOR_TEXT_MUTED, FONT_MONO
)


class SingleActuatorBar(QFrame):
    """
    Compact individual Actuator Level card with single-row header:
    [ L1 ]  [ 331.7 mm ]  [ Δ +0.0 mm ]                 [ SEGURO ]
    [=====================|======================================]
    220 mm                  332 mm                         420 mm
    """
    def __init__(self, index: int, parent=None):
        super().__init__(parent)
        self.index = index
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 5px;
                padding: 3px 6px;
            }}
        """)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(3)

        # 1. Compact Header Row: L{i} + Value + Delta + Status aligned right
        hdr_layout = QHBoxLayout()
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        hdr_layout.setSpacing(8)

        # Short identifier: L1, L2, ...
        self.lbl_id = QLabel(f"L{index + 1}")
        self.lbl_id.setStyleSheet(f"font-weight: 800; font-size: 13px; color: {COLOR_CYAN};")
        hdr_layout.addWidget(self.lbl_id)

        # Live Length Readout (immediately right of L{i})
        self.lbl_val = QLabel(f"{Q_NOMINAL:.1f} mm")
        self.lbl_val.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 13px;
            font-weight: 700;
            color: {COLOR_TEXT_PRIMARY};
        """)
        hdr_layout.addWidget(self.lbl_val)

        # Delta from nominal position
        self.lbl_delta = QLabel("Δ +0.0 mm")
        self.lbl_delta.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 10px;
            color: {COLOR_TEXT_MUTED};
        """)
        hdr_layout.addWidget(self.lbl_delta)

        hdr_layout.addStretch()

        # Safety State (aligned to far right)
        self.lbl_status = QLabel("SEGURO")
        self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
        hdr_layout.addWidget(self.lbl_status)

        layout.addLayout(hdr_layout)

        # 2. Progress / Level Bar
        self.bar = QProgressBar()
        self.bar.setRange(int(Q_MIN * 10), int(Q_MAX * 10))
        self.bar.setValue(int(Q_NOMINAL * 10))
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(8)
        self._set_bar_color(COLOR_CYAN)
        layout.addWidget(self.bar)

        # 3. Numeric Limit Indicators (Min, Nom, Max)
        lim_layout = QHBoxLayout()
        lim_layout.setContentsMargins(0, 0, 0, 0)
        lbl_min = QLabel(f"{Q_MIN:.0f} mm")
        lbl_nom = QLabel(f"Nom: {Q_NOMINAL:.0f} mm")
        lbl_max = QLabel(f"{Q_MAX:.0f} mm")
        
        for l in (lbl_min, lbl_nom, lbl_max):
            l.setStyleSheet(f"font-size: 9px; color: {COLOR_TEXT_MUTED}; font-family: {FONT_MONO};")
            
        lim_layout.addWidget(lbl_min)
        lim_layout.addStretch()
        lim_layout.addWidget(lbl_nom)
        lim_layout.addStretch()
        lim_layout.addWidget(lbl_max)
        layout.addLayout(lim_layout)

    def _set_bar_color(self, hex_color: str):
        self.bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #0E141B;
                border: 1px solid {COLOR_BORDER};
                border-radius: 3px;
            }}
            QProgressBar::chunk {{
                background-color: {hex_color};
                border-radius: 2px;
            }}
        """)

    def update_length(self, q_val: float):
        """Update live readouts, progress bar, and safety classification."""
        self.lbl_val.setText(f"{q_val:.1f} mm")
        delta = q_val - Q_NOMINAL
        sign = "+" if delta >= 0 else ""
        self.lbl_delta.setText(f"Δ {sign}{delta:.1f} mm")

        # Clamp bar display value
        int_val = int(np.clip(q_val * 10, Q_MIN * 10, Q_MAX * 10))
        self.bar.setValue(int_val)

        # Safety boundary evaluation
        if q_val < Q_MIN or q_val > Q_MAX:
            self._set_bar_color(COLOR_EMERGENCY)
            self.lbl_status.setText("LÍMITE EXCEDIDO")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERGENCY};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_EMERGENCY};")
        elif q_val < Q_SAFE_MIN or q_val > Q_SAFE_MAX:
            self._set_bar_color(COLOR_WARNING)
            self.lbl_status.setText("CERCA DE LÍMITE")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_WARNING};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_WARNING};")
        else:
            self._set_bar_color(COLOR_CYAN)
            self.lbl_status.setText("SEGURO")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_TEXT_PRIMARY};")


class ActuatorClusterWidget(QWidget):
    """
    Container displaying all 6 actuator status cards in a high-density vertical layout.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(6)

        self.bars: List[SingleActuatorBar] = []
        for i in range(6):
            bar = SingleActuatorBar(i, self)
            self.bars.append(bar)
            layout.addWidget(bar)

    def update_actuators(self, q_mm: Union[list, np.ndarray]):
        for i, val in enumerate(q_mm[:6]):
            self.bars[i].update_length(float(val))
