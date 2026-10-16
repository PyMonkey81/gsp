"""
Real-time 20-30 Hz multi-actuator telemetry plotting widget using pyqtgraph.PlotWidget.
"""

from collections import deque
from typing import List, Union
import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton, QFrame, QCheckBox
)
from PySide6.QtCore import Qt, QTimer
import pyqtgraph as pg

from ui.styles import (
    COLOR_CARD_BG, COLOR_BORDER, COLOR_CYAN, COLOR_EMERALD,
    COLOR_WARNING, COLOR_TEXT_PRIMARY, COLOR_TEXT_MUTED
)

# Actuator channel colors
ACTUATOR_COLORS = [
    "#00E5FF",  # L1: Cyan
    "#00E676",  # L2: Emerald
    "#FF9100",  # L3: Orange
    "#FF4081",  # L4: Pink/Magenta
    "#FFD600",  # L5: Yellow
    "#448AFF",  # L6: Neon Blue
]


class TelemetryPlotWidget(QWidget):
    """
    High-performance real-time oscilloscope plotting actuator lengths, tracking errors, or control efforts.
    """
    def __init__(self, buffer_size: int = 250, parent=None):
        super().__init__(parent)
        self.buffer_size = buffer_size
        self._target_q = np.array([332.0] * 6, dtype=np.float64)
        self._actual_q = np.array([332.0] * 6, dtype=np.float64)

        # Ring buffers for history: time index, and 6 actuator values
        self.history_len = buffer_size
        self.time_buffer = deque(maxlen=self.history_len)
        self.data_buffers = [deque(maxlen=self.history_len) for _ in range(6)]
        self._sample_counter = 0

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # Plot Controls Header
        ctrl_bar = QFrame()
        ctrl_bar.setStyleSheet(f"background-color: {COLOR_CARD_BG}; border-bottom: 1px solid {COLOR_BORDER}; padding: 2px;")
        ctrl_layout = QHBoxLayout(ctrl_bar)
        ctrl_layout.setContentsMargins(6, 2, 6, 2)
        ctrl_layout.setSpacing(6)

        title = QLabel("TELEMETRÍA EN TIEMPO REAL")
        title.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_CYAN};")
        ctrl_layout.addWidget(title)

        ctrl_layout.addStretch()

        self.combo_mode = QComboBox()
        self.combo_mode.addItems([
            "Longitudes Actuales (mm)",
            "Error de Seguimiento e_i (mm)",
            "Esfuerzo Estimado Sliding-PI"
        ])
        self.combo_mode.setFixedHeight(22)
        ctrl_layout.addWidget(self.combo_mode)

        self.chk_autoscale = QCheckBox("Auto-escala")
        self.chk_autoscale.setChecked(True)
        self.chk_autoscale.setStyleSheet(f"color: {COLOR_TEXT_MUTED}; font-size: 10px;")
        ctrl_layout.addWidget(self.chk_autoscale)

        btn_clear = QPushButton("Limpiar")
        btn_clear.setFixedHeight(22)
        btn_clear.clicked.connect(self.clear_buffers)
        ctrl_layout.addWidget(btn_clear)

        layout.addWidget(ctrl_bar)

        # Configure pyqtgraph PlotWidget
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground("#0E1318")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel("left", "Magnitud", units="mm", color="#90A4AE")
        self.plot_widget.setLabel("bottom", "Muestras", color="#90A4AE")

        # Create 6 curves with legend
        self.legend = self.plot_widget.addLegend(offset=(10, 10))
        self.legend.setBrush(pg.mkBrush(20, 26, 34, 180))
        self.legend.setPen(pg.mkPen(COLOR_BORDER))

        self.curves = []
        for i in range(6):
            pen = pg.mkPen(color=ACTUATOR_COLORS[i], width=1.8)
            curve = self.plot_widget.plot(name=f"L{i+1}", pen=pen)
            self.curves.append(curve)

        layout.addWidget(self.plot_widget)

        # Plot refresh timer (runs at 30 Hz)
        self.render_timer = QTimer(self)
        self.render_timer.setInterval(33)  # 30 Hz
        self.render_timer.timeout.connect(self._render_curves)
        self.render_timer.start()

    def update_telemetry(self, target_q: Union[list, np.ndarray], actual_q: Union[list, np.ndarray]):
        """
        Ingest new telemetry sample.
        """
        self._target_q = np.asarray(target_q, dtype=np.float64)
        self._actual_q = np.asarray(actual_q, dtype=np.float64)

        mode_idx = self.combo_mode.currentIndex()
        if mode_idx == 0:
            # Actual lengths
            vals = self._actual_q
        elif mode_idx == 1:
            # Tracking error e_i = target - actual
            vals = self._target_q - self._actual_q
        else:
            # Control effort proxy: sliding surface s = e_dot + lambda * e
            e = self._target_q - self._actual_q
            vals = np.tanh(e * 0.4) * 100.0  # Normalized % control signal

        self._sample_counter += 1
        self.time_buffer.append(self._sample_counter)
        for i in range(6):
            self.data_buffers[i].append(float(vals[i]))

    def _render_curves(self):
        """Update pyqtgraph curve data buffers."""
        if not self.time_buffer:
            return

        x = np.array(self.time_buffer)
        for i in range(6):
            if len(self.data_buffers[i]) == len(x):
                self.curves[i].setData(x, np.array(self.data_buffers[i]))

        if self.chk_autoscale.isChecked():
            self.plot_widget.enableAutoRange(axis='y', enable=True)

    def clear_buffers(self):
        self.time_buffer.clear()
        for b in self.data_buffers:
            b.clear()
        for c in self.curves:
            c.clear()
