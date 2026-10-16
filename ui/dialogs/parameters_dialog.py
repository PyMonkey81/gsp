"""
ParametersDialog for Gough-Stewart 6-DOF Platform.
Allows editing of geometry, actuator limits, Az/Bz anchor matrices, controller gains, and trajectory parameters.
Saves to config/parameters.json.
"""

import json
import re
from copy import deepcopy
from pathlib import Path
from typing import Dict, Any, List

import numpy as np
from PySide6.QtCore import Qt, Signal, QEvent
from PySide6.QtGui import QGuiApplication, QKeySequence
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QDoubleSpinBox, QLabel, QPushButton, QScrollArea, QWidget,
    QTabWidget, QTableWidget, QHeaderView, QMessageBox, QAbstractItemView
)

from config.parameters import (
    L0, STROKE, ACTUATOR_HOME_PERCENT, ALPHA_0, D, RT,
    ALPHA_PI, KP_S, KI_S, T_HOME_END, T_TRACKING_END,
    DEMO_DURATION, DT, FILTER_WN, TRAJ_POINTS, Az, Bz
)
from ui.styles import (
    COLOR_BG_DARK, COLOR_PANEL_BG, COLOR_CARD_BG, COLOR_BORDER,
    COLOR_BORDER_LIGHT, COLOR_CYAN, COLOR_EMERALD, COLOR_EMERGENCY,
    COLOR_WARNING, COLOR_TEXT_PRIMARY, COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED, FONT_MAIN, FONT_MONO
)

SETTINGS_PATH = Path(__file__).resolve().parents[2] / "config" / "parameters.json"


def get_default_parameters() -> Dict[str, Any]:
    return {
        "L0": float(L0),
        "STROKE": float(STROKE),
        "ACTUATOR_HOME_PERCENT": float(ACTUATOR_HOME_PERCENT),
        "ALPHA_0": float(ALPHA_0),
        "D": [float(x) for x in D],
        "RT": float(RT),
        "ALPHA_PI": float(ALPHA_PI),
        "KP_S": float(KP_S),
        "KI_S": float(KI_S),
        "T_HOME_END": float(T_HOME_END),
        "T_TRACKING_END": float(T_TRACKING_END),
        "DEMO_DURATION": float(DEMO_DURATION),
        "DT": float(DT),
        "FILTER_WN": float(FILTER_WN),
        "TRAJ_POINTS": [[float(a), float(b)] for a, b in TRAJ_POINTS],
        "Az": [[float(x) for x in pt] for pt in Az],
        "Bz": [[float(x) for x in pt] for pt in Bz]
    }


def load_parameters() -> Dict[str, Any]:
    defaults = get_default_parameters()
    if SETTINGS_PATH.exists():
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, dict):
                defaults.update(data)
        except Exception:
            pass
    return defaults


def save_parameters(params: Dict[str, Any]) -> Dict[str, Any]:
    settings = deepcopy(get_default_parameters())
    if isinstance(params, dict):
        settings.update(params)

    SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(SETTINGS_PATH, "w", encoding="utf-8") as fh:
        json.dump(settings, fh, indent=2, ensure_ascii=False)
    return settings


class MatrixTableWidget(QTableWidget):
    """6x3 Table (X, Y, Z) for anchor points Az or Bz with clipboard support."""

    ROWS = 6
    COLS = 3
    ROW_HEIGHT = 28

    def __init__(self, parent=None):
        super().__init__(self.ROWS, self.COLS, parent)
        self.spinboxes: List[List[QDoubleSpinBox]] = [[None] * self.COLS for _ in range(self.ROWS)]
        self._build()

    def _build(self):
        self.setHorizontalHeaderLabels(["X (mm)", "Y (mm)", "Z (mm)"])
        self.setVerticalHeaderLabels([f"P{i + 1}" for i in range(self.ROWS)])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)
        self.verticalHeader().setDefaultSectionSize(self.ROW_HEIGHT)
        self.setWordWrap(False)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QAbstractItemView.NoSelection)

        prev = None
        for row in range(self.ROWS):
            for col in range(self.COLS):
                spin = QDoubleSpinBox()
                spin.setRange(-600.0, 600.0)
                spin.setDecimals(2)
                spin.setSingleStep(0.5)
                spin.setStyleSheet(f"""
                    QDoubleSpinBox {{
                        background-color: {COLOR_CARD_BG};
                        color: {COLOR_TEXT_PRIMARY};
                        border: 1px solid {COLOR_BORDER};
                        border-radius: 3px;
                        padding: 2px 4px;
                        font-family: {FONT_MONO};
                        font-size: 11px;
                    }}
                    QDoubleSpinBox:focus {{
                        border-color: {COLOR_CYAN};
                    }}
                """)
                spin.installEventFilter(self)
                self.setCellWidget(row, col, spin)
                self.spinboxes[row][col] = spin
                if prev is not None:
                    self.setTabOrder(prev, spin)
                prev = spin

        header_h = self.horizontalHeader().height() or 26
        total_h = header_h + self.ROW_HEIGHT * self.ROWS + 2 * self.frameWidth() + 4
        self.setFixedHeight(total_h)

    def _find_index(self, widget):
        for r in range(self.ROWS):
            for c in range(self.COLS):
                if self.spinboxes[r][c] is widget:
                    return r, c
        return None

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress:
            if event.matches(QKeySequence.Paste):
                self._paste_from_clipboard(obj)
                return True
            if event.key() in (Qt.Key_Return, Qt.Key_Enter):
                pos = self._find_index(obj)
                if pos:
                    r, c = pos
                    nr, nc = (r + 1, c) if r + 1 < self.ROWS else (0, c)
                    self.spinboxes[nr][nc].setFocus()
                    self.spinboxes[nr][nc].selectAll()
                return True
        return super().eventFilter(obj, event)

    def _paste_from_clipboard(self, origin_widget):
        pos = self._find_index(origin_widget)
        start_row, start_col = pos if pos else (0, 0)

        text = QGuiApplication.clipboard().text()
        lines = [line for line in text.splitlines() if line.strip() != ""]
        for i, line in enumerate(lines):
            row = start_row + i
            if row >= self.ROWS:
                break
            cells = re.split(r"[\t,]", line)
            for j, cell in enumerate(cells):
                col = start_col + j
                if col >= self.COLS:
                    break
                try:
                    self.spinboxes[row][col].setValue(float(cell.strip()))
                except ValueError:
                    continue

    def get_values(self) -> List[List[float]]:
        return [[float(self.spinboxes[r][c].value()) for c in range(self.COLS)] for r in range(self.ROWS)]

    def set_values(self, values: List[List[float]]):
        for r in range(min(self.ROWS, len(values))):
            for c in range(min(self.COLS, len(values[r]))):
                self.spinboxes[r][c].setValue(float(values[r][c]))

    def to_tsv(self) -> str:
        lines = []
        for r in range(self.ROWS):
            lines.append("\t".join(f"{self.spinboxes[r][c].value():.2f}" for c in range(self.COLS)))
        return "\n".join(lines)


class ParametersDialog(QDialog):
    """
    Modal dialog for configuring Gough-Stewart parameters:
    - Actuator geometry (L0, STROKE, HOME %)
    - Mobile/Base anchor matrices (Az, Bz)
    - Controller gains (ALPHA_PI, KP_S, KI_S)
    - Trajectory settings (T_HOME, T_TRACKING, DT, FILTER_WN)
    """
    parameters_changed = Signal(dict)

    def __init__(self, current_params: Dict[str, Any] = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configuración de Parámetros del Sistema")
        self.setMinimumSize(780, 700)
        self.resize(840, 750)
        self.setModal(True)

        self.setStyleSheet(f"""
            QDialog {{
                background-color: {COLOR_BG_DARK};
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_MAIN};
            }}
            QLabel {{
                color: {COLOR_TEXT_PRIMARY};
                font-size: 11px;
            }}
            QGroupBox {{
                background-color: {COLOR_PANEL_BG};
                color: {COLOR_CYAN};
                font-weight: 700;
                font-size: 11px;
                border: 1px solid {COLOR_BORDER};
                border-radius: 6px;
                margin-top: 14px;
                padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
            }}
            QDoubleSpinBox, QSpinBox {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 5px 8px;
                min-width: 100px;
                font-family: {FONT_MONO};
                font-size: 11px;
            }}
            QDoubleSpinBox:focus, QSpinBox:focus {{
                border-color: {COLOR_CYAN};
            }}
            QPushButton {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 7px 16px;
                font-weight: 600;
                font-size: 11px;
            }}
            QPushButton:hover {{
                background-color: #222C38;
                border-color: {COLOR_CYAN};
            }}
            QPushButton#btn_ok {{
                background-color: #1B5E20;
                border: 1px solid #2E7D32;
                color: #FFFFFF;
                font-weight: 700;
            }}
            QPushButton#btn_ok:hover {{
                background-color: #2E7D32;
            }}
            QTableWidget {{
                background-color: {COLOR_PANEL_BG};
                color: {COLOR_TEXT_PRIMARY};
                gridline-color: {COLOR_BORDER};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
            }}
            QHeaderView::section {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_SECONDARY};
                padding: 4px;
                border: 1px solid {COLOR_BORDER};
                font-weight: 700;
                font-size: 10px;
            }}
            QTabWidget::pane {{
                border: 1px solid {COLOR_BORDER};
                background: {COLOR_PANEL_BG};
                border-radius: 4px;
            }}
            QTabBar::tab {{
                background: {COLOR_CARD_BG};
                color: {COLOR_TEXT_SECONDARY};
                padding: 8px 18px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-weight: 600;
            }}
            QTabBar::tab:selected {{
                background: {COLOR_PANEL_BG};
                color: {COLOR_CYAN};
                border-bottom: 2px solid {COLOR_CYAN};
            }}
            QScrollArea {{
                border: none;
                background: transparent;
            }}
        """)

        self.params = load_parameters()
        if current_params:
            self.params.update(current_params)

        self._build_ui()
        self._load_values()

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # -------------------- TAB 1: Actuadores & Geometría --------------------
        tab1_scroll = QScrollArea()
        tab1_scroll.setWidgetResizable(True)
        tab1_widget = QWidget()
        tab1_layout = QVBoxLayout(tab1_widget)
        tab1_layout.setSpacing(10)
        tab1_scroll.setWidget(tab1_widget)
        tabs.addTab(tab1_scroll, "Actuadores y Geometría")

        # Actuadores
        gb_act = QGroupBox("Límites de Carrera y Actuadores")
        form_act = QFormLayout(gb_act)
        form_act.setSpacing(8)

        self.sp_l0 = self._spin(100.0, 500.0, 2, 0.5)
        self.sp_stroke = self._spin(10.0, 300.0, 1, 1.0)
        self.sp_home_pct = self._spin(0.0, 100.0, 1, 1.0)

        form_act.addRow("Longitud retraída L0 (mm):", self.sp_l0)
        form_act.addRow("Carrera útil STROKE (mm):", self.sp_stroke)
        form_act.addRow("Posición HOME (% carrera):", self.sp_home_pct)

        self.lbl_min_max = QLabel("")
        self.lbl_min_max.setStyleSheet(f"color: {COLOR_EMERALD}; font-weight: 700; font-family: {FONT_MONO}; font-size: 11px;")
        form_act.addRow("Rango Actuadores:", self.lbl_min_max)

        self.sp_l0.valueChanged.connect(self._update_min_max)
        self.sp_stroke.valueChanged.connect(self._update_min_max)
        tab1_layout.addWidget(gb_act)

        # Geometría de plataforma
        gb_geo = QGroupBox("Geometría y Origen de Plataforma")
        form_geo = QFormLayout(gb_geo)
        form_geo.setSpacing(8)

        self.sp_alpha0 = self._spin(-180.0, 180.0, 4, 0.05)
        self.sp_rt = self._spin(0.0, 500.0, 1, 1.0)
        self.sp_dx = self._spin(-200.0, 200.0, 2, 0.5)
        self.sp_dy = self._spin(-200.0, 200.0, 2, 0.5)
        self.sp_dz = self._spin(0.0, 1000.0, 2, 1.0)

        form_geo.addRow("ALPHA_0 (rad):", self.sp_alpha0)
        form_geo.addRow("Radio offset RT (mm):", self.sp_rt)
        form_geo.addRow("Centro D.x (mm):", self.sp_dx)
        form_geo.addRow("Centro D.y (mm):", self.sp_dy)
        form_geo.addRow("Altura D.z (mm):", self.sp_dz)
        tab1_layout.addWidget(gb_geo)

        # Matrices Az y Bz
        gb_mat = QGroupBox("Matrices de Anclaje Az (Móvil) y Bz (Base)")
        mat_layout = QVBoxLayout(gb_mat)
        mat_layout.setSpacing(6)

        info_mat = QLabel("Coordenadas 3D (X, Y, Z) en mm para los 6 pares de anclajes:")
        info_mat.setStyleSheet(f"color: {COLOR_TEXT_SECONDARY}; font-size: 10px;")
        mat_layout.addWidget(info_mat)

        row_az = QHBoxLayout()
        lbl_az = QLabel("Az – Plataforma Móvil (6x3):")
        lbl_az.setStyleSheet(f"font-weight: 700; color: {COLOR_CYAN};")
        row_az.addWidget(lbl_az)
        row_az.addStretch()
        btn_copy_az = QPushButton("Copiar Az")
        btn_copy_az.setFixedHeight(22)
        btn_copy_az.clicked.connect(lambda: QGuiApplication.clipboard().setText(self.table_az.to_tsv()))
        row_az.addWidget(btn_copy_az)
        mat_layout.addLayout(row_az)

        self.table_az = MatrixTableWidget()
        mat_layout.addWidget(self.table_az)

        row_bz = QHBoxLayout()
        lbl_bz = QLabel("Bz – Base Fija (6x3):")
        lbl_bz.setStyleSheet(f"font-weight: 700; color: {COLOR_CYAN};")
        row_bz.addWidget(lbl_bz)
        row_bz.addStretch()
        btn_copy_bz = QPushButton("Copiar Bz")
        btn_copy_bz.setFixedHeight(22)
        btn_copy_bz.clicked.connect(lambda: QGuiApplication.clipboard().setText(self.table_bz.to_tsv()))
        row_bz.addWidget(btn_copy_bz)
        mat_layout.addLayout(row_bz)

        self.table_bz = MatrixTableWidget()
        mat_layout.addWidget(self.table_bz)

        tab1_layout.addWidget(gb_mat)
        tab1_layout.addStretch()

        # -------------------- TAB 2: Controlador --------------------
        tab2 = QWidget()
        tab2_layout = QVBoxLayout(tab2)
        tab2_layout.setSpacing(10)
        tabs.addTab(tab2, "Controlador")

        gb_ctrl = QGroupBox("Ganancias del Controlador Sliding-PI")
        form_ctrl = QFormLayout(gb_ctrl)
        form_ctrl.setSpacing(8)

        self.sp_alpha_pi = self._spin(1.0, 1000.0, 1, 5.0)
        self.sp_kp = self._spin(10.0, 100000.0, 1, 100.0)
        self.sp_ki = self._spin(0.1, 10000.0, 1, 10.0)

        form_ctrl.addRow("ALPHA_PI (Superficie de deslizamiento):", self.sp_alpha_pi)
        form_ctrl.addRow("KP_S (Ganancia Proporcional):", self.sp_kp)
        form_ctrl.addRow("KI_S (Ganancia Integral):", self.sp_ki)

        tab2_layout.addWidget(gb_ctrl)
        tab2_layout.addStretch()

        # -------------------- TAB 3: Trayectoria --------------------
        tab3 = QWidget()
        tab3_layout = QVBoxLayout(tab3)
        tab3_layout.setSpacing(10)
        tabs.addTab(tab3, "Trayectoria")

        gb_time = QGroupBox("Tiempos de Secuencia y Filtro")
        form_time = QFormLayout(gb_time)
        form_time.setSpacing(8)

        self.sp_t_home = self._spin(1.0, 300.0, 1, 1.0)
        self.sp_t_track = self._spin(5.0, 2000.0, 1, 5.0)
        self.sp_dt = self._spin(0.0001, 0.1, 4, 0.001)
        self.sp_filter = self._spin(0.1, 50.0, 2, 0.2)
        self.sp_demo_duration = self._spin(5.0, 1200.0, 1, 5.0)

        form_time.addRow("T_HOME_END (s) [Duración Approach]:", self.sp_t_home)
        form_time.addRow("T_TRACKING_END (s) [Fin de Tracking]:", self.sp_t_track)
        form_time.addRow("DT (s) [Paso de integración]:", self.sp_dt)
        form_time.addRow("FILTER_WN [Frecuencia natural filtro]:", self.sp_filter)
        form_time.addRow("DEMO_DURATION (s) [Duración total demo]:", self.sp_demo_duration)
        tab3_layout.addWidget(gb_time)

        # Puntos de trayectoria por defecto (3 pares α, β en grados)
        gb_traj = QGroupBox("Waypoints de Trayectoria Predeterminada (α Azimut, β Elevación en °)")
        traj_layout = QVBoxLayout(gb_traj)
        traj_layout.setSpacing(6)

        note_traj = QLabel("3 puntos clave para Spline: P1 (Approach), P2 (Pico central), P3 (Retorno):")
        note_traj.setStyleSheet(f"color: {COLOR_TEXT_SECONDARY}; font-size: 10px;")
        traj_layout.addWidget(note_traj)

        self.table_traj = QTableWidget(3, 2)
        self.table_traj.setHorizontalHeaderLabels(["Alpha Azimut (°)", "Beta Elevación (°)"])
        self.table_traj.setVerticalHeaderLabels(["Punto 1", "Punto 2", "Punto 3"])
        self.table_traj.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_traj.setFixedHeight(120)

        for row in range(3):
            for col in range(2):
                spin = QDoubleSpinBox()
                spin.setRange(-180.0, 180.0)
                spin.setDecimals(1)
                spin.setSingleStep(0.5)
                spin.setStyleSheet(f"""
                    QDoubleSpinBox {{
                        background-color: {COLOR_CARD_BG};
                        color: {COLOR_TEXT_PRIMARY};
                        border: 1px solid {COLOR_BORDER};
                        border-radius: 3px;
                        padding: 2px 4px;
                        font-family: {FONT_MONO};
                        font-size: 11px;
                    }}
                """)
                self.table_traj.setCellWidget(row, col, spin)

        traj_layout.addWidget(self.table_traj)
        tab3_layout.addWidget(gb_traj)
        tab3_layout.addStretch()

        # -------------------- BOTONES FINALES --------------------
        btn_bar = QHBoxLayout()
        btn_bar.setSpacing(10)

        btn_defaults = QPushButton("↺ Restaurar Defaults")
        btn_defaults.clicked.connect(self._reset_defaults)
        btn_bar.addWidget(btn_defaults)

        btn_bar.addStretch()

        btn_cancel = QPushButton("Cancelar")
        btn_cancel.clicked.connect(self.reject)
        btn_bar.addWidget(btn_cancel)

        btn_ok = QPushButton("✔ Aplicar y Guardar")
        btn_ok.setObjectName("btn_ok")
        btn_ok.clicked.connect(self._on_save_clicked)
        btn_bar.addWidget(btn_ok)

        main_layout.addLayout(btn_bar)

    def _spin(self, min_val: float, max_val: float, decimals: int, step: float) -> QDoubleSpinBox:
        sp = QDoubleSpinBox()
        sp.setRange(min_val, max_val)
        sp.setDecimals(decimals)
        sp.setSingleStep(step)
        return sp

    def _update_min_max(self):
        l0 = self.sp_l0.value()
        stroke = self.sp_stroke.value()
        amin = l0
        amax = l0 + stroke
        self.lbl_min_max.setText(f"MIN = {amin:.1f} mm  |  MAX = {amax:.1f} mm")

    def _load_values(self):
        p = self.params

        self.sp_l0.setValue(float(p.get("L0", L0)))
        self.sp_stroke.setValue(float(p.get("STROKE", STROKE)))
        self.sp_home_pct.setValue(float(p.get("ACTUATOR_HOME_PERCENT", ACTUATOR_HOME_PERCENT)))
        self._update_min_max()

        self.sp_alpha0.setValue(float(p.get("ALPHA_0", ALPHA_0)))
        self.sp_rt.setValue(float(p.get("RT", RT)))

        d_vals = p.get("D", [0.0, 0.0, 331.94])
        self.sp_dx.setValue(float(d_vals[0]))
        self.sp_dy.setValue(float(d_vals[1]))
        self.sp_dz.setValue(float(d_vals[2]))

        if "Az" in p:
            self.table_az.set_values(p["Az"])
        if "Bz" in p:
            self.table_bz.set_values(p["Bz"])

        self.sp_alpha_pi.setValue(float(p.get("ALPHA_PI", ALPHA_PI)))
        self.sp_kp.setValue(float(p.get("KP_S", KP_S)))
        self.sp_ki.setValue(float(p.get("KI_S", KI_S)))

        self.sp_t_home.setValue(float(p.get("T_HOME_END", T_HOME_END)))
        self.sp_t_track.setValue(float(p.get("T_TRACKING_END", T_TRACKING_END)))
        self.sp_dt.setValue(float(p.get("DT", DT)))
        self.sp_filter.setValue(float(p.get("FILTER_WN", FILTER_WN)))
        self.sp_demo_duration.setValue(float(p.get("DEMO_DURATION", DEMO_DURATION)))

        traj_pts = p.get("TRAJ_POINTS", TRAJ_POINTS)
        for r in range(min(3, len(traj_pts))):
            for c in range(2):
                widget = self.table_traj.cellWidget(r, c)
                if isinstance(widget, QDoubleSpinBox):
                    widget.setValue(float(traj_pts[r][c]))

    def _reset_defaults(self):
        self.params = get_default_parameters()
        self._load_values()

    def _on_save_clicked(self):
        try:
            traj_pts = []
            for r in range(3):
                row_vals = []
                for c in range(2):
                    w = self.table_traj.cellWidget(r, c)
                    row_vals.append(float(w.value()) if isinstance(w, QDoubleSpinBox) else 0.0)
                traj_pts.append(row_vals)

            updated = {
                "L0": float(self.sp_l0.value()),
                "STROKE": float(self.sp_stroke.value()),
                "ACTUATOR_HOME_PERCENT": float(self.sp_home_pct.value()),
                "ALPHA_0": float(self.sp_alpha0.value()),
                "D": [float(self.sp_dx.value()), float(self.sp_dy.value()), float(self.sp_dz.value())],
                "RT": float(self.sp_rt.value()),
                "Az": self.table_az.get_values(),
                "Bz": self.table_bz.get_values(),
                "ALPHA_PI": float(self.sp_alpha_pi.value()),
                "KP_S": float(self.sp_kp.value()),
                "KI_S": float(self.sp_ki.value()),
                "T_HOME_END": float(self.sp_t_home.value()),
                "T_TRACKING_END": float(self.sp_t_track.value()),
                "DT": float(self.sp_dt.value()),
                "FILTER_WN": float(self.sp_filter.value()),
                "DEMO_DURATION": float(self.sp_demo_duration.value()),
                "TRAJ_POINTS": traj_pts
            }

            save_parameters(updated)
            self.parameters_changed.emit(updated)
            QMessageBox.information(self, "Parámetros Guardados", "Los parámetros del sistema se han guardado exitosamente en parameters.json.")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Error Guardando Parámetros", f"Ocurrió un error al guardar los parámetros:\n{e}")
