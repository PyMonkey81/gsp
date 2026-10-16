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

import config.parameters as _params
from config.parameters import SAFE_MARGIN
from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_CYAN, COLOR_EMERALD,
    COLOR_EMERGENCY, COLOR_WARNING, COLOR_TEXT_PRIMARY,
    COLOR_TEXT_MUTED, FONT_MONO
)


class SingleActuatorBar(QFrame):
    """
    Compact individual Actuator Level card with single-row header:
    [ L1 ]  [ 0.0 mm ]  [ 0.0% | L: 247.4 mm ]             [ HOME (0 mm) ]
    [=====================|======================================]
    HOME: 0 mm             50 mm                         MAX: 100 mm
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

        # 1. Compact Header Row: L{i} + Extension (0..100 mm) + % / Total Length + Status aligned right
        hdr_layout = QHBoxLayout()
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        hdr_layout.setSpacing(8)

        # Short identifier: L1, L2, ...
        self.lbl_id = QLabel(f"L{index + 1}")
        self.lbl_id.setStyleSheet(f"font-weight: 800; font-size: 13px; color: {COLOR_CYAN};")
        hdr_layout.addWidget(self.lbl_id)

        # Live Extension Readout (0.0 mm at HOME when fully retracted)
        self.lbl_val = QLabel("0.0 mm")
        self.lbl_val.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 13px;
            font-weight: 700;
            color: {COLOR_TEXT_PRIMARY};
        """)
        hdr_layout.addWidget(self.lbl_val)

        # Stroke percentage and total anchor-to-anchor length
        self.lbl_delta = QLabel(f"0.0% (L: {_params.L0:.1f} mm)")
        self.lbl_delta.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 10px;
            color: {COLOR_TEXT_MUTED};
        """)
        hdr_layout.addWidget(self.lbl_delta)

        hdr_layout.addStretch()

        # Safety State (aligned to far right)
        self.lbl_status = QLabel("HOME (0 mm)")
        self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
        hdr_layout.addWidget(self.lbl_status)

        layout.addLayout(hdr_layout)

        # 2. Progress / Level Bar (0.0 mm to STROKE mm in tenths of mm)
        stroke = float(_params.STROKE)
        self.bar = QProgressBar()
        self.bar.setRange(0, int(stroke * 10))
        self.bar.setValue(0)
        self.bar.setTextVisible(False)
        self.bar.setFixedHeight(8)
        self._set_bar_color(COLOR_CYAN)
        layout.addWidget(self.bar)

        # 3. Numeric Limit Indicators (HOME: 0 mm, Mid, MAX: STROKE mm)
        lim_layout = QHBoxLayout()
        lim_layout.setContentsMargins(0, 0, 0, 0)
        self.lbl_min = QLabel("HOME: 0 mm")
        self.lbl_nom = QLabel(f"Media: {stroke * 0.5:.0f} mm")
        self.lbl_max = QLabel(f"MAX: {stroke:.0f} mm")
        
        for l in (self.lbl_min, self.lbl_nom, self.lbl_max):
            l.setStyleSheet(f"font-size: 9px; color: {COLOR_TEXT_MUTED}; font-family: {FONT_MONO};")
            
        lim_layout.addWidget(self.lbl_min)
        lim_layout.addStretch()
        lim_layout.addWidget(self.lbl_nom)
        lim_layout.addStretch()
        lim_layout.addWidget(self.lbl_max)
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
        """
        Update live readouts, progress bar, and safety classification.
        Accepts either total anchor-to-anchor length q_mm (e.g. ~247.41..347.41 mm)
        or effective extension q_ef = q_mm - L0 (0.0..100.0 mm).
        At HOME (retracted), extension is 0.0 mm (0.0%).
        """
        l0 = float(_params.L0)
        stroke = float(_params.STROKE)

        # Distinguish total anchor-to-anchor length vs direct extension
        if q_val > stroke + 40.0:
            total_mm = float(q_val)
            ext_mm = total_mm - l0
        else:
            ext_mm = float(q_val)
            total_mm = l0 + ext_mm

        # Snap tiny floating-point noise around 0.0 mm
        if abs(ext_mm) < 0.05:
            ext_mm = 0.0

        pct = np.clip((ext_mm / max(stroke, 1e-6)) * 100.0, 0.0, 100.0)

        self.lbl_val.setText(f"{ext_mm:.1f} mm")
        self.lbl_delta.setText(f"{pct:.1f}% (L: {total_mm:.1f} mm)")

        # Update range if STROKE was modified dynamically
        max_bar = max(10, int(stroke * 10))
        if self.bar.maximum() != max_bar:
            self.bar.setRange(0, max_bar)
            self.lbl_nom.setText(f"Media: {stroke * 0.5:.0f} mm")
            self.lbl_max.setText(f"MAX: {stroke:.0f} mm")

        # Clamp bar display value
        int_val = int(np.clip(round(ext_mm * 10.0), 0, max_bar))
        self.bar.setValue(int_val)

        # Safety boundary evaluation in extension space [0, STROKE] mm
        tol = 0.5
        if ext_mm < -tol or ext_mm > stroke + tol:
            self._set_bar_color(COLOR_EMERGENCY)
            self.lbl_status.setText("LÍMITE EXCEDIDO")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERGENCY};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_EMERGENCY};")
        elif ext_mm > stroke - SAFE_MARGIN:
            self._set_bar_color(COLOR_WARNING)
            self.lbl_status.setText("CERCA DE LÍMITE")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_WARNING};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_WARNING};")
        elif abs(ext_mm) <= 0.15:
            self._set_bar_color(COLOR_CYAN)
            self.lbl_status.setText("HOME (0 mm)")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
            self.lbl_val.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 13px; font-weight: 700; color: {COLOR_TEXT_PRIMARY};")
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
