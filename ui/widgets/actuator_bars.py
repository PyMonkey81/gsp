"""
Actuator level display widgets for 6-DOF Gough-Stewart Platform.
Redesigned with independent setpoint sliders and separate real feedback indicators.
"""

from typing import List, Union
import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar, QSlider, QSpinBox, QFrame
)
from PySide6.QtCore import Qt, Signal

import config.parameters as _params
from config.parameters import SAFE_MARGIN
from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_BORDER_LIGHT, COLOR_CYAN, COLOR_EMERALD,
    COLOR_EMERGENCY, COLOR_WARNING, COLOR_TEXT_PRIMARY,
    COLOR_TEXT_MUTED, FONT_MONO
)


class SingleActuatorBar(QFrame):
    """
    Individual Actuator Card with:
    - Compact Header: L{i} | SP: % (mm) | Real: % (mm) | Safety Status
    - Independent Setpoint Slider (0–100%) + QSpinBox
    - Separate Real Position Feedback Bar (Emerald)
    - Limits guide: 0% (0 mm) ... 50% ... 100% (STROKE mm)
    """
    slider_changed = Signal(int, int)  # (index: 0..5, value: 0..100)

    def __init__(self, index: int, parent=None):
        super().__init__(parent)
        self.index = index
        self._stroke = float(_params.STROKE)
        self._l0 = float(_params.L0)

        self.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 6px;
                padding: 4px 8px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        # 1. Header Row: L{i} + SP: % (mm) + L: mm + Real: % (mm) + Status
        hdr_layout = QHBoxLayout()
        hdr_layout.setContentsMargins(0, 0, 0, 0)
        hdr_layout.setSpacing(8)

        self.lbl_id = QLabel(f"L{index + 1}")
        self.lbl_id.setStyleSheet(f"font-weight: 800; font-size: 13px; color: {COLOR_CYAN};")
        hdr_layout.addWidget(self.lbl_id)

        # Setpoint Readout
        self.lbl_sp = QLabel("SP: 0% (0.0 mm)")
        self.lbl_sp.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 12px;
            font-weight: 700;
            color: {COLOR_TEXT_PRIMARY};
        """)
        hdr_layout.addWidget(self.lbl_sp)

        self.lbl_sp_len = QLabel(f"(L: {self._l0:.1f} mm)")
        self.lbl_sp_len.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 10px;
            color: {COLOR_TEXT_MUTED};
        """)
        hdr_layout.addWidget(self.lbl_sp_len)

        # Separator dot
        lbl_dot = QLabel("•")
        lbl_dot.setStyleSheet(f"color: {COLOR_BORDER_LIGHT}; font-size: 10px;")
        hdr_layout.addWidget(lbl_dot)

        # Real Position Readout (shown separately, never overwritten by slider)
        self.lbl_real = QLabel("Real: --")
        self.lbl_real.setStyleSheet(f"""
            font-family: {FONT_MONO};
            font-size: 11px;
            font-weight: 700;
            color: {COLOR_EMERALD};
        """)
        hdr_layout.addWidget(self.lbl_real)

        hdr_layout.addStretch()

        # Safety State Tag (aligned to far right)
        self.lbl_status = QLabel("HOME (0 mm)")
        self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
        hdr_layout.addWidget(self.lbl_status)

        layout.addLayout(hdr_layout)

        # 2. Setpoint Control Row: Slider (0..100) + SpinBox (0..100)
        ctrl_layout = QHBoxLayout()
        ctrl_layout.setContentsMargins(0, 0, 0, 0)
        ctrl_layout.setSpacing(8)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(0)
        self.slider.setTracking(True)
        self.slider.setFixedHeight(22)
        ctrl_layout.addWidget(self.slider)

        self.spin = QSpinBox()
        self.spin.setRange(0, 100)
        self.spin.setValue(0)
        self.spin.setSuffix(" %")
        self.spin.setFixedWidth(68)
        self.spin.setFixedHeight(22)
        self.spin.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 11px;")
        ctrl_layout.addWidget(self.spin)

        layout.addLayout(ctrl_layout)

        # 3. Real Position Feedback Bar (Emerald Green, distinct from Cyan Setpoint)
        real_layout = QHBoxLayout()
        real_layout.setContentsMargins(0, 0, 0, 0)
        real_layout.setSpacing(6)

        lbl_real_tag = QLabel("Real:")
        lbl_real_tag.setFixedWidth(28)
        lbl_real_tag.setStyleSheet(f"font-size: 9px; font-weight: 600; color: {COLOR_EMERALD};")
        real_layout.addWidget(lbl_real_tag)

        self.real_bar = QProgressBar()
        self.real_bar.setRange(0, 100)
        self.real_bar.setValue(0)
        self.real_bar.setTextVisible(False)
        self.real_bar.setFixedHeight(8)
        self.real_bar.setToolTip(f"Posición REAL medida de actuador L{index + 1} (relleno verde)")
        self.real_bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: #0E141B;
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
            }}
            QProgressBar::chunk {{
                background-color: {COLOR_EMERALD};
                border-radius: 3px;
            }}
        """)
        real_layout.addWidget(self.real_bar)

        # Spacer to align with spinbox in row above
        real_layout.addSpacing(74)
        layout.addLayout(real_layout)

        # 4. Limits Indicator Row
        lim_layout = QHBoxLayout()
        lim_layout.setContentsMargins(34, 0, 74, 0)
        self.lbl_min = QLabel("0% (0 mm)")
        self.lbl_nom = QLabel(f"50% ({self._stroke * 0.5:.0f} mm)")
        self.lbl_max = QLabel(f"100% ({self._stroke:.0f} mm)")

        for l in (self.lbl_min, self.lbl_nom, self.lbl_max):
            l.setStyleSheet(f"font-size: 9px; color: {COLOR_TEXT_MUTED}; font-family: {FONT_MONO};")

        lim_layout.addWidget(self.lbl_min)
        lim_layout.addStretch()
        lim_layout.addWidget(self.lbl_nom)
        lim_layout.addStretch()
        lim_layout.addWidget(self.lbl_max)
        layout.addLayout(lim_layout)

        # Connect coupled slider and spinbox
        self.slider.valueChanged.connect(self._on_slider_value_changed)
        self.spin.valueChanged.connect(self._on_spin_value_changed)

    def _on_slider_value_changed(self, val: int):
        self.spin.blockSignals(True)
        self.spin.setValue(val)
        self.spin.blockSignals(False)
        self._update_setpoint_display(float(val))
        self.slider_changed.emit(self.index, val)

    def _on_spin_value_changed(self, val: int):
        self.slider.blockSignals(True)
        self.slider.setValue(val)
        self.slider.blockSignals(False)
        self._update_setpoint_display(float(val))
        self.slider_changed.emit(self.index, val)

    def _update_setpoint_display(self, pct: float, total_mm: float = None):
        stroke = float(_params.STROKE)
        l0 = float(_params.L0)
        ext_mm = stroke * (pct / 100.0)
        if total_mm is None:
            total_mm = l0 + ext_mm

        self.lbl_sp.setText(f"SP: {pct:.0f}% ({ext_mm:.1f} mm)")
        self.lbl_sp_len.setText(f"(L: {total_mm:.1f} mm)")

        # Safety evaluation
        tol = 0.5
        if ext_mm < -tol or ext_mm > stroke + tol:
            self.lbl_status.setText("LÍMITE EXCEDIDO")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERGENCY};")
            self.lbl_sp.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 12px; font-weight: 700; color: {COLOR_EMERGENCY};")
        elif ext_mm > stroke - SAFE_MARGIN:
            self.lbl_status.setText("CERCA DE LÍMITE")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_WARNING};")
            self.lbl_sp.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 12px; font-weight: 700; color: {COLOR_WARNING};")
        elif abs(ext_mm) <= 0.15:
            self.lbl_status.setText("HOME (0 mm)")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
            self.lbl_sp.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 12px; font-weight: 700; color: {COLOR_TEXT_PRIMARY};")
        else:
            self.lbl_status.setText("SEGURO")
            self.lbl_status.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_EMERALD};")
            self.lbl_sp.setStyleSheet(f"font-family: {FONT_MONO}; font-size: 12px; font-weight: 700; color: {COLOR_TEXT_PRIMARY};")

    def set_setpoint(self, pct: float, total_mm: float = None):
        """Actualiza el slider y setpoint de la UI con señales bloqueadas (no emite comando serial)."""
        int_val = int(round(np.clip(pct, 0.0, 100.0)))
        self.slider.blockSignals(True)
        self.spin.blockSignals(True)
        self.slider.setValue(int_val)
        self.spin.setValue(int_val)
        self.slider.blockSignals(False)
        self.spin.blockSignals(False)
        self._update_setpoint_display(float(pct), total_mm)

    def set_real_position(self, real_pct: float):
        """
        Actualiza únicamente la lectura y barra real del hardware.
        No pisa el setpoint ni altera la posición del slider.
        """
        r_val = int(round(np.clip(real_pct, 0.0, 100.0)))
        self.real_bar.setValue(r_val)
        stroke = float(_params.STROKE)
        real_ext_mm = stroke * (r_val / 100.0)
        self.lbl_real.setText(f"Real: {r_val}% ({real_ext_mm:.1f} mm)")

    def sync_hardware(self, pct_val: float):
        """
        Sincroniza setpoint y lectura real con el hardware recibido de getpos sin emitir movimiento.
        """
        self.set_real_position(pct_val)
        self.set_setpoint(pct_val)

    def update_length(self, q_val: float):
        """
        Compatibilidad con la cinemática del sistema (da/R -> q_mm).
        Actualiza el setpoint del actuador sin enviar comandos de un solo eje.
        """
        l0 = float(_params.L0)
        stroke = float(_params.STROKE)

        if q_val > stroke + 40.0:
            total_mm = float(q_val)
            ext_mm = total_mm - l0
        else:
            ext_mm = float(q_val)
            total_mm = l0 + ext_mm

        if abs(ext_mm) < 0.05:
            ext_mm = 0.0

        pct = np.clip((ext_mm / max(stroke, 1e-6)) * 100.0, 0.0, 100.0)
        self.set_setpoint(pct, total_mm)


class ActuatorClusterWidget(QWidget):
    """
    Contenedor de los 6 controles independientes de actuador (L1..L6).
    """
    actuator_moved = Signal(int, int)  # (index: 0..5, value: 0..100)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(6)

        self.bars: List[SingleActuatorBar] = []
        for i in range(6):
            bar = SingleActuatorBar(i, self)
            bar.slider_changed.connect(self._on_bar_slider_changed)
            self.bars.append(bar)
            layout.addWidget(bar)

    def _on_bar_slider_changed(self, index: int, val: int):
        self.actuator_moved.emit(index, val)

    def update_actuators(self, q_mm: Union[list, np.ndarray]):
        """Actualiza setpoints desde cinemática."""
        for i, val in enumerate(q_mm[:6]):
            self.bars[i].update_length(float(val))

    def update_real_positions(self, actual_pcts: Union[list, np.ndarray]):
        """Actualiza lecturas reales de telemetría sin pisar los sliders."""
        for i, val in enumerate(actual_pcts[:6]):
            self.bars[i].set_real_position(float(val))

    def sync_hardware_positions(self, pcts: Union[list, np.ndarray], q_mm=None):
        """Sincroniza UI (lectura real y sliders) con la posición real del hardware tras getpos."""
        for i, val in enumerate(pcts[:6]):
            self.bars[i].sync_hardware(float(val))

    def sync_setpoint_sliders(self, pcts: Union[list, np.ndarray]):
        """Asegura que los sliders arranquen en la posición actual de cada eje al entrar a la pestaña."""
        for i, val in enumerate(pcts[:6]):
            self.bars[i].set_setpoint(float(val))
