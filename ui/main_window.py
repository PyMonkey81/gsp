"""
Main Application Window for 6-DOF Gough-Stewart Platform GUI.
Integrates 3D OpenGL viewer, Cartesian/Track kinematics, real-time telemetry, and asynchronous serial manager.
"""

import sys
import time
from typing import Optional
import numpy as np

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QTabWidget,
    QGroupBox, QLabel, QPushButton, QSlider, QDoubleSpinBox, QComboBox,
    QCheckBox, QTextEdit, QProgressBar, QFrame, QFileDialog, QMessageBox, QStatusBar
)
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QFont

from config.parameters import (
    Z_HOME, Q_MIN, Q_MAX, Q_NOMINAL,
    LIMITS_X, LIMITS_Y, LIMITS_Z, LIMITS_ROLL, LIMITS_PITCH, LIMITS_YAW,
    LIMITS_ALPHA, LIMITS_BETA, AVAILABLE_BAUDRATES, DEFAULT_BAUDRATE,
    BASE_JOINTS, PLATFORM_JOINTS
)
from core.kinematics import (
    pose_to_q, inverse_kinematics, InvJac, calculate_singularity_metrics, SimpleCubicSpline
)
from core.serial_manager import SerialManager
from ui.styles import (
    COLOR_CYAN, COLOR_EMERALD, COLOR_EMERGENCY, COLOR_WARNING,
    COLOR_BORDER, COLOR_CARD_BG, COLOR_PANEL_BG, FONT_MONO
)
from ui.widgets.led_indicator import StatusBadge
from ui.widgets.actuator_bars import ActuatorClusterWidget
from ui.widgets.gl_view import Platform3DView
from ui.widgets.telemetry_plot import TelemetryPlotWidget


class MainWindow(QMainWindow):
    """
    Main Application Window orchestrating kinematics, serial telemetry, 3D visualization,
    and trajectory tracking.
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gough-Stewart 6-DOF Platform | Trajectory Tracking & Kinematics Control")
        self.resize(1440, 920)
        self.setMinimumSize(1100, 720)

        # Initialize Serial Communication Manager (Asynchronous worker in QThread)
        self.serial_manager = SerialManager()

        # State Variables
        self.current_da = np.array([0.0, 0.0, Z_HOME], dtype=np.float64)
        self.current_R = np.eye(3, dtype=np.float64)
        self.current_q = np.array([Q_NOMINAL] * 6, dtype=np.float64)
        self.actual_q = np.array([Q_NOMINAL] * 6, dtype=np.float64)
        self.control_mode = "CARTESIAN"  # 'CARTESIAN', 'TRACK', or 'IDLE'

        # Trajectory Tracking State
        self.trajectory_spline: Optional[SimpleCubicSpline] = None
        self.traj_timer = QTimer(self)
        self.traj_timer.setInterval(33)  # ~30 Hz execution loop
        self.traj_start_time = 0.0
        self.traj_pause_time = 0.0
        self.traj_elapsed = 0.0
        self.traj_duration = 10.0
        self.is_tracking = False
        self.is_traj_paused = False

        # Build UI Structure
        self._setup_ui()
        self._connect_signals()

        # Perform initial kinematics calculation
        self._on_cartesian_changed()

    def _setup_ui(self):
        """Construct the entire interface layout."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        # 1. TOP BAR
        self._build_top_bar()
        main_layout.addWidget(self.top_bar)

        # Splitters for responsive multi-panel layout
        main_vsplitter = QSplitter(Qt.Orientation.Vertical)
        main_vsplitter.setHandleWidth(4)

        # TOP HALF: [Central-Left 3D Viewer] + [Right Kinematic Tabs]
        top_hsplitter = QSplitter(Qt.Orientation.Horizontal)
        top_hsplitter.setHandleWidth(4)

        # 2. Visor 3D Central-Izquierdo
        self.gl_view = Platform3DView(base_joints=BASE_JOINTS, platform_joints=PLATFORM_JOINTS)
        top_hsplitter.addWidget(self.gl_view)

        # 3. Panel Derecho con QTabWidget
        self._build_right_control_tabs()
        top_hsplitter.addWidget(self.tab_widget)
        top_hsplitter.setSizes([920, 480])
        top_hsplitter.setStretchFactor(0, 1)  # gl_view takes maximum horizontal space
        top_hsplitter.setStretchFactor(1, 0)

        main_vsplitter.addWidget(top_hsplitter)

        # BOTTOM HALF: [Telemetry & Jacobians] + [Sequences & Logs]
        bottom_hsplitter = QSplitter(Qt.Orientation.Horizontal)
        bottom_hsplitter.setHandleWidth(4)

        # 4. Panel Inferior-Izquierdo: Telemetría y Jacobiano
        self._build_bottom_left_panel()
        bottom_hsplitter.addWidget(self.bottom_left_panel)

        # 5. Panel Inferior-Derecho: Secuencias & Logs
        self._build_bottom_right_panel()
        bottom_hsplitter.addWidget(self.bottom_right_panel)
        bottom_hsplitter.setSizes([850, 550])

        main_vsplitter.addWidget(bottom_hsplitter)
        main_vsplitter.setSizes([600, 260])
        main_vsplitter.setStretchFactor(0, 1)  # top 3D section takes maximum vertical space
        main_vsplitter.setStretchFactor(1, 0)

        main_layout.addWidget(main_vsplitter)

        # Status Bar
        self._build_status_bar()

    # =========================================================================
    # 1. TOP BAR
    # =========================================================================
    def _build_top_bar(self):
        self.top_bar = QFrame()
        self.top_bar.setObjectName("TopBar")
        layout = QHBoxLayout(self.top_bar)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(10)

        # Serial Port Selector
        lbl_port = QLabel("PUERTO:")
        lbl_port.setStyleSheet(f"font-weight: 700; font-size: 10px; color: {COLOR_CYAN};")
        layout.addWidget(lbl_port)

        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(180)
        self.refresh_ports()
        layout.addWidget(self.combo_ports)

        btn_refresh = QPushButton("↻")
        btn_refresh.setFixedWidth(28)
        btn_refresh.setToolTip("Refrescar puertos disponibles")
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)

        # Baudrate Selector
        self.combo_baud = QComboBox()
        for b in AVAILABLE_BAUDRATES:
            self.combo_baud.addItem(str(b))
        self.combo_baud.setCurrentText(str(DEFAULT_BAUDRATE))
        self.combo_baud.setFixedWidth(85)
        layout.addWidget(self.combo_baud)

        # Connect / Disconnect Button
        self.btn_connect = QPushButton("CONECTAR")
        self.btn_connect.setObjectName("PrimaryButton")
        self.btn_connect.setFixedWidth(110)
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        layout.addSpacing(12)

        # Status Badges
        self.badge_conn = StatusBadge("ESTADO", "DESCONECTADO", "red")
        self.badge_mode = StatusBadge("MODO", "CARTESIANO", "cyan")
        self.badge_safe = StatusBadge("SEGURIDAD", "MODO SEGURO", "green")
        layout.addWidget(self.badge_conn)
        layout.addWidget(self.badge_mode)
        layout.addWidget(self.badge_safe)

        layout.addStretch()

        # Stream serial toggle
        self.chk_stream = QCheckBox("Stream Serial")
        self.chk_stream.setChecked(True)
        self.chk_stream.setStyleSheet("font-size: 10px; font-weight: 600; color: #90A4AE;")
        layout.addWidget(self.chk_stream)

        # Home Button
        self.btn_home = QPushButton("⌂ HOME")
        self.btn_home.setToolTip("Retornar a posición segura nominal")
        self.btn_home.clicked.connect(self.go_home)
        layout.addWidget(self.btn_home)

        # Pause Button
        self.btn_pause = QPushButton("⏸ PAUSA")
        self.btn_pause.setObjectName("WarningButton")
        self.btn_pause.clicked.connect(self.toggle_pause)
        layout.addWidget(self.btn_pause)

        # Emergency Stop Button
        self.btn_estop = QPushButton("⛔ PARADA EMERGENCIA")
        self.btn_estop.setObjectName("EmergencyButton")
        self.btn_estop.clicked.connect(self.trigger_emergency_stop)
        layout.addWidget(self.btn_estop)

    # =========================================================================
    # 3. RIGHT PANEL (Control Cinemático con QTabWidget)
    # =========================================================================
    def _build_right_control_tabs(self):
        self.tab_widget = QTabWidget()

        # TAB 1: Modo Cartesiano (6-DOF)
        self.tab_cartesian = QWidget()
        self._build_tab_cartesian()
        self.tab_widget.addTab(self.tab_cartesian, "Modo Cartesiano (6-DOF)")

        # TAB 2: Modo Track (Alpha / Beta)
        self.tab_track = QWidget()
        self._build_tab_track()
        self.tab_widget.addTab(self.tab_track, "Modo Track (Alpha / Beta)")

        # TAB 3: Actuadores (L1 - L6)
        self.tab_actuators = QWidget()
        self._build_tab_actuators()
        self.tab_widget.addTab(self.tab_actuators, "Actuadores (L1 - L6)")

        self.tab_widget.currentChanged.connect(self._on_tab_changed)

    def _build_tab_cartesian(self):
        layout = QVBoxLayout(self.tab_cartesian)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Position Group: X, Y, Z
        grp_pos = QGroupBox("Posición Cartesiana (mm)")
        pos_layout = QVBoxLayout(grp_pos)
        pos_layout.setSpacing(6)

        self.sliders_pos = {}
        self.spins_pos = {}

        pos_configs = [
            ("X", LIMITS_X[0], LIMITS_X[1], 0.0, "mm"),
            ("Y", LIMITS_Y[0], LIMITS_Y[1], 0.0, "mm"),
            ("Z", LIMITS_Z[0], LIMITS_Z[1], Z_HOME, "mm")
        ]

        for axis, min_v, max_v, init_v, unit in pos_configs:
            row = QHBoxLayout()
            lbl = QLabel(f"{axis}:")
            lbl.setFixedWidth(20)
            lbl.setStyleSheet("font-weight: 700; color: #ECEFF1;")

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_v * 10), int(max_v * 10))
            slider.setValue(int(init_v * 10))

            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setValue(init_v)
            spin.setSuffix(f" {unit}")
            spin.setFixedWidth(85)

            # Coupled slider and spinbox
            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val / 10.0))
            spin.valueChanged.connect(lambda val, sl=slider: sl.setValue(int(val * 10.0)))
            spin.valueChanged.connect(self._on_cartesian_changed)

            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(spin)
            pos_layout.addLayout(row)

            self.sliders_pos[axis] = slider
            self.spins_pos[axis] = spin

        layout.addWidget(grp_pos)

        # Orientation Group: Roll, Pitch, Yaw
        grp_rot = QGroupBox("Orientación Angular (grados)")
        rot_layout = QVBoxLayout(grp_rot)
        rot_layout.setSpacing(6)

        self.sliders_rot = {}
        self.spins_rot = {}

        rot_configs = [
            ("Roll", LIMITS_ROLL[0], LIMITS_ROLL[1], 0.0, "°"),
            ("Pitch", LIMITS_PITCH[0], LIMITS_PITCH[1], 0.0, "°"),
            ("Yaw", LIMITS_YAW[0], LIMITS_YAW[1], 0.0, "°")
        ]

        for axis, min_v, max_v, init_v, unit in rot_configs:
            row = QHBoxLayout()
            lbl = QLabel(f"{axis[:1]}:")
            lbl.setToolTip(axis)
            lbl.setFixedWidth(20)
            lbl.setStyleSheet("font-weight: 700; color: #ECEFF1;")

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(min_v * 10), int(max_v * 10))
            slider.setValue(int(init_v * 10))

            spin = QDoubleSpinBox()
            spin.setRange(min_v, max_v)
            spin.setDecimals(1)
            spin.setSingleStep(0.5)
            spin.setValue(init_v)
            spin.setSuffix(f" {unit}")
            spin.setFixedWidth(85)

            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val / 10.0))
            spin.valueChanged.connect(lambda val, sl=slider: sl.setValue(int(val * 10.0)))
            spin.valueChanged.connect(self._on_cartesian_changed)

            row.addWidget(lbl)
            row.addWidget(slider)
            row.addWidget(spin)
            rot_layout.addLayout(row)

            self.sliders_rot[axis] = slider
            self.spins_rot[axis] = spin

        layout.addWidget(grp_rot)

        # Quick Actions
        btn_box = QHBoxLayout()
        btn_zero = QPushButton("Centrar XY / Cero")
        btn_zero.clicked.connect(self._reset_cartesian_zero)
        btn_home_tab = QPushButton("Posición Home")
        btn_home_tab.clicked.connect(self.go_home)
        btn_box.addWidget(btn_zero)
        btn_box.addWidget(btn_home_tab)
        layout.addLayout(btn_box)

        layout.addStretch()

    def _build_tab_track(self):
        layout = QVBoxLayout(self.tab_track)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # Pointing Angle Controls: Alpha (Azimuth) & Beta (Elevation)
        grp_polar = QGroupBox("Orientación Polar / Espacio de Tarea")
        polar_layout = QVBoxLayout(grp_polar)
        polar_layout.setSpacing(8)

        # Alpha (Azimuth)
        row_alpha = QHBoxLayout()
        lbl_a = QLabel("Alpha (α):")
        lbl_a.setStyleSheet("font-weight: 700; color: #00E5FF;")
        lbl_a.setFixedWidth(70)

        self.slider_alpha = QSlider(Qt.Orientation.Horizontal)
        self.slider_alpha.setRange(int(LIMITS_ALPHA[0]), int(LIMITS_ALPHA[1]))
        self.slider_alpha.setValue(0)

        self.spin_alpha = QDoubleSpinBox()
        self.spin_alpha.setRange(LIMITS_ALPHA[0], LIMITS_ALPHA[1])
        self.spin_alpha.setDecimals(1)
        self.spin_alpha.setSuffix(" ° (Azim)")
        self.spin_alpha.setFixedWidth(100)

        self.slider_alpha.valueChanged.connect(lambda val: self.spin_alpha.setValue(float(val)))
        self.spin_alpha.valueChanged.connect(lambda val: self.slider_alpha.setValue(int(val)))
        self.spin_alpha.valueChanged.connect(self._on_track_angles_changed)

        row_alpha.addWidget(lbl_a)
        row_alpha.addWidget(self.slider_alpha)
        row_alpha.addWidget(self.spin_alpha)
        polar_layout.addLayout(row_alpha)

        # Beta (Elevation / Tilt)
        row_beta = QHBoxLayout()
        lbl_b = QLabel("Beta (β):")
        lbl_b.setStyleSheet("font-weight: 700; color: #00E5FF;")
        lbl_b.setFixedWidth(70)

        self.slider_beta = QSlider(Qt.Orientation.Horizontal)
        self.slider_beta.setRange(int(LIMITS_BETA[0] * 10), int(LIMITS_BETA[1] * 10))
        self.slider_beta.setValue(0)

        self.spin_beta = QDoubleSpinBox()
        self.spin_beta.setRange(LIMITS_BETA[0], LIMITS_BETA[1])
        self.spin_beta.setDecimals(1)
        self.spin_beta.setSuffix(" ° (Elev)")
        self.spin_beta.setFixedWidth(100)

        self.slider_beta.valueChanged.connect(lambda val: self.spin_beta.setValue(val / 10.0))
        self.spin_beta.valueChanged.connect(lambda val: self.slider_beta.setValue(int(val * 10.0)))
        self.spin_beta.valueChanged.connect(self._on_track_angles_changed)

        row_beta.addWidget(lbl_b)
        row_beta.addWidget(self.slider_beta)
        row_beta.addWidget(self.spin_beta)
        polar_layout.addLayout(row_beta)

        layout.addWidget(grp_polar)

        # Trajectory Generator & Spline Tracking Group
        grp_traj = QGroupBox("Generador de Trayectorias (SimpleCubicSpline)")
        traj_layout = QVBoxLayout(grp_traj)
        traj_layout.setSpacing(8)

        # Profile Selector
        row_prof = QHBoxLayout()
        lbl_prof = QLabel("Perfil:")
        lbl_prof.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.combo_traj_pattern = QComboBox()
        self.combo_traj_pattern.addItems([
            "Precesión Cónica (Círculo Alpha 360°)",
            "Figura en 8 (Lemniscata Polar)",
            "Espiral de Escaneo (Scan Spiral)",
            "Respuesta al Escalón (Step Test)",
            "Waypoints Personalizados (CSV)"
        ])
        btn_load_csv = QPushButton("Cargar CSV...")
        btn_load_csv.setFixedHeight(22)
        btn_load_csv.clicked.connect(self._load_custom_waypoints)

        row_prof.addWidget(lbl_prof)
        row_prof.addWidget(self.combo_traj_pattern)
        row_prof.addWidget(btn_load_csv)
        traj_layout.addLayout(row_prof)

        # Period & Duration Sliders
        row_params = QHBoxLayout()
        lbl_dur = QLabel("Duración:")
        lbl_dur.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.spin_traj_dur = QDoubleSpinBox()
        self.spin_traj_dur.setRange(3.0, 120.0)
        self.spin_traj_dur.setValue(12.0)
        self.spin_traj_dur.setSuffix(" s")
        self.spin_traj_dur.setFixedWidth(75)

        lbl_amp = QLabel("Amplitud β:")
        lbl_amp.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.spin_traj_amp = QDoubleSpinBox()
        self.spin_traj_amp.setRange(2.0, 30.0)
        self.spin_traj_amp.setValue(15.0)
        self.spin_traj_amp.setSuffix(" °")
        self.spin_traj_amp.setFixedWidth(75)

        row_params.addWidget(lbl_dur)
        row_params.addWidget(self.spin_traj_dur)
        row_params.addSpacing(10)
        row_params.addWidget(lbl_amp)
        row_params.addWidget(self.spin_traj_amp)
        traj_layout.addLayout(row_params)

        # Execution Progress Bar
        self.traj_progress = QProgressBar()
        self.traj_progress.setRange(0, 100)
        self.traj_progress.setValue(0)
        self.traj_progress.setFormat("Progreso: %p%")
        traj_layout.addWidget(self.traj_progress)

        # Execution Buttons: [START TRACKING], [PAUSE TRACK], [ABORT]
        row_exec = QHBoxLayout()
        self.btn_start_track = QPushButton("▶ INICIAR TRACKING")
        self.btn_start_track.setObjectName("SuccessButton")
        self.btn_start_track.clicked.connect(self.start_trajectory_tracking)

        self.btn_pause_track = QPushButton("⏸ PAUSA TRACK")
        self.btn_pause_track.setObjectName("WarningButton")
        self.btn_pause_track.clicked.connect(self.toggle_pause_trajectory)

        self.btn_abort_track = QPushButton("⏹ ABORTAR")
        self.btn_abort_track.setObjectName("EmergencyButton")
        self.btn_abort_track.clicked.connect(self.abort_trajectory)

        row_exec.addWidget(self.btn_start_track)
        row_exec.addWidget(self.btn_pause_track)
        row_exec.addWidget(self.btn_abort_track)
        traj_layout.addLayout(row_exec)

        layout.addWidget(grp_traj)
        layout.addStretch()

    def _build_tab_actuators(self):
        layout = QVBoxLayout(self.tab_actuators)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        self.actuator_cluster = ActuatorClusterWidget()
        layout.addWidget(self.actuator_cluster)
        layout.addStretch()

    # =========================================================================
    # 4. PANEL INFERIOR-IZQUIERDO (Telemetría y Jacobiano)
    # =========================================================================
    def _build_bottom_left_panel(self):
        self.bottom_left_panel = QFrame()
        self.bottom_left_panel.setObjectName("PanelCard")
        layout = QHBoxLayout(self.bottom_left_panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # Multi-channel Telemetry Plot
        self.telemetry_plot = TelemetryPlotWidget()
        layout.addWidget(self.telemetry_plot, stretch=3)

        # Subpanel "Workspace & Jacobians"
        jac_box = QGroupBox("Espacio de Trabajo & Jacobiano")
        jac_box.setMinimumWidth(260)
        jac_layout = QVBoxLayout(jac_box)
        jac_layout.setContentsMargins(8, 8, 8, 8)
        jac_layout.setSpacing(6)

        # Singularity / Manipulability Indicators
        row_manip = QHBoxLayout()
        lbl_m = QLabel("Manipulabilidad w:")
        lbl_m.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.lbl_manip_val = QLabel("0.00")
        self.lbl_manip_val.setStyleSheet(f"font-family: {FONT_MONO}; font-weight: 700; color: {COLOR_CYAN};")
        row_manip.addWidget(lbl_m)
        row_manip.addStretch()
        row_manip.addWidget(self.lbl_manip_val)
        jac_layout.addLayout(row_manip)

        row_cond = QHBoxLayout()
        lbl_c = QLabel("Condición κ(J⁻¹):")
        lbl_c.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.lbl_cond_val = QLabel("1.00")
        self.lbl_cond_val.setStyleSheet(f"font-family: {FONT_MONO}; font-weight: 700; color: {COLOR_CYAN};")
        row_cond.addWidget(lbl_c)
        row_cond.addStretch()
        row_cond.addWidget(self.lbl_cond_val)
        jac_layout.addLayout(row_cond)

        self.lbl_singularity_status = QLabel("ESTADO: ÓPTIMO")
        self.lbl_singularity_status.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_EMERALD};")
        jac_layout.addWidget(self.lbl_singularity_status)

        # Transformation Matrix Text Display
        lbl_mat = QLabel("Matriz de Transformación [R | da]:")
        lbl_mat.setStyleSheet("color: #90A4AE; font-size: 10px; font-weight: 600;")
        jac_layout.addWidget(lbl_mat)

        self.txt_matrix = QLabel()
        self.txt_matrix.setObjectName("MatrixDisplay")
        self.txt_matrix.setFixedHeight(85)
        self.txt_matrix.setText("[ 1.000  0.000  0.000 |   0.0 ]\n[ 0.000  1.000  0.000 |   0.0 ]\n[ 0.000  0.000  1.000 | 300.0 ]")
        jac_layout.addWidget(self.txt_matrix)

        layout.addWidget(jac_box, stretch=2)

    # =========================================================================
    # 5. PANEL INFERIOR-DERECHO (Secuencias, Calibración & Logs)
    # =========================================================================
    def _build_bottom_right_panel(self):
        self.bottom_right_panel = QFrame()
        self.bottom_right_panel.setObjectName("PanelCard")
        layout = QVBoxLayout(self.bottom_right_panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Action Buttons Header: [CALIBRATE], [RUN PATTERN], [ZERO SENSORS]
        btn_bar = QHBoxLayout()
        btn_bar.setSpacing(6)

        self.btn_calibrate = QPushButton("⚙ CALIBRAR")
        self.btn_calibrate.clicked.connect(self.calibrate_platform)
        btn_bar.addWidget(self.btn_calibrate)

        self.btn_pattern = QPushButton("⚡ PATRÓN TEST")
        self.btn_pattern.clicked.connect(self.run_test_pattern)
        btn_bar.addWidget(self.btn_pattern)

        self.btn_zero_sensors = QPushButton("⌖ CERAR SENSORES")
        self.btn_zero_sensors.clicked.connect(self.zero_sensors)
        btn_bar.addWidget(self.btn_zero_sensors)

        btn_clear_log = QPushButton("Limpiar Log")
        btn_clear_log.setFixedHeight(22)
        btn_clear_log.clicked.connect(self._clear_logs)
        btn_bar.addWidget(btn_clear_log)

        layout.addLayout(btn_bar)

        # Monospace Terminal Console Log
        self.txt_console = QTextEdit()
        self.txt_console.setObjectName("ConsoleLog")
        self.txt_console.setReadOnly(True)
        layout.addWidget(self.txt_console)

        self.log_message("[INIT] Sistema de Control Cinemático Gough-Stewart iniciado.")
        self.log_message("[INFO] Geometría cargada: Base R=250mm, Plataforma R=150mm.")

    # =========================================================================
    # STATUS BAR
    # =========================================================================
    def _build_status_bar(self):
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        self.lbl_sb_pose = QLabel("Pose: X: 0.0 Y: 0.0 Z: 300.0 | R: 0.0° P: 0.0° Y: 0.0°")
        self.lbl_sb_pose.setStyleSheet("color: #ECEFF1; font-family: monospace; font-size: 11px;")

        self.lbl_sb_stream = QLabel("Streaming: ON")
        self.lbl_sb_stream.setStyleSheet(f"color: {COLOR_CYAN}; font-weight: 700; font-size: 11px;")

        self.lbl_sb_fps = QLabel("Loop: 30 Hz")
        self.lbl_sb_fps.setStyleSheet("color: #90A4AE; font-size: 11px;")

        self.status_bar.addWidget(self.lbl_sb_pose, stretch=1)
        self.status_bar.addPermanentWidget(self.lbl_sb_stream)
        self.status_bar.addPermanentWidget(self.lbl_sb_fps)

    # =========================================================================
    # SIGNAL CONNECTIONS & SERIAL WIRING
    # =========================================================================
    def _connect_signals(self):
        # SerialManager signals
        self.serial_manager.connected.connect(self._on_serial_connected)
        self.serial_manager.disconnected.connect(self._on_serial_disconnected)
        self.serial_manager.telemetry_received.connect(self._on_telemetry_received)
        self.serial_manager.message_received.connect(self.log_message)
        self.serial_manager.error_occurred.connect(self.log_error)
        self.serial_manager.safe_mode_changed.connect(self._on_safe_mode_changed)

        # Trajectory execution timer
        self.traj_timer.timeout.connect(self._trajectory_tick)

    # =========================================================================
    # SERIAL ACTIONS & PORTS
    # =========================================================================
    def refresh_ports(self):
        current = self.combo_ports.currentText()
        self.combo_ports.clear()
        ports = self.serial_manager.available_ports()
        self.combo_ports.addItems(ports)
        if current in ports:
            self.combo_ports.setCurrentText(current)

    def _toggle_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect_port()
        else:
            port = self.combo_ports.currentText()
            baud = int(self.combo_baud.currentText())
            if not port:
                self.log_error("[SERIAL] No hay puerto seleccionado.")
                return
            self.serial_manager.connect_port(port, baud)

    @Slot(str, int)
    def _on_serial_connected(self, port: str, baud: int):
        self.btn_connect.setText("DESCONECTAR")
        self.btn_connect.setStyleSheet(f"background-color: {COLOR_EMERGENCY}; border: 1px solid #FF5252;")
        self.badge_conn.set_status("CONECTADO", "green")
        self.log_message(f"[SERIAL] Conectado a {port} a {baud} baudios.")

    @Slot()
    def _on_serial_disconnected(self):
        self.btn_connect.setText("CONECTAR")
        self.btn_connect.setStyleSheet("")
        self.badge_conn.set_status("DESCONECTADO", "red")
        self.log_message("[SERIAL] Desconectado.")

    @Slot(bool)
    def _on_safe_mode_changed(self, is_safe: bool):
        if is_safe:
            self.badge_safe.set_status("MODO SEGURO", "green")
        else:
            self.badge_safe.set_status("ESTOP ACTIVO", "red")

    @Slot(list)
    def _on_telemetry_received(self, actual_vals: list):
        self.actual_q = np.array(actual_vals, dtype=np.float64)
        # Update telemetry plot with target vs actual
        self.telemetry_plot.update_telemetry(self.current_q, self.actual_q)

    # =========================================================================
    # KINEMATICS & CONTROL DISPATCH
    # =========================================================================
    def _on_tab_changed(self, index: int):
        if index == 0:
            self.control_mode = "CARTESIAN"
            self.badge_mode.set_status("CARTESIANO", "cyan")
        elif index == 1:
            self.control_mode = "TRACK"
            self.badge_mode.set_status("TRACK (α/β)", "cyan")
        else:
            self.control_mode = "ACTUATORS"
            self.badge_mode.set_status("MONITOR L1-L6", "amber")

    def _on_cartesian_changed(self):
        """Called when any Cartesian slider / spinbox changes."""
        if not hasattr(self, 'spins_pos') or not hasattr(self, 'spins_rot'):
            return

        x = self.spins_pos["X"].value()
        y = self.spins_pos["Y"].value()
        z = self.spins_pos["Z"].value()
        roll = self.spins_rot["Roll"].value()
        pitch = self.spins_rot["Pitch"].value()
        yaw = self.spins_rot["Yaw"].value()

        # Compute Inverse Kinematics
        q_mm, da, R = pose_to_q(x, y, z, roll, pitch, yaw)
        self._apply_kinematics_result(q_mm, da, R)

        self.lbl_sb_pose.setText(
            f"Pose: X:{x:+.1f} Y:{y:+.1f} Z:{z:.1f} | R:{roll:+.1f}° P:{pitch:+.1f}° Y:{yaw:+.1f}°"
        )

    def _on_track_angles_changed(self):
        """Called when Alpha or Beta slider / spinbox changes."""
        if self.is_tracking:
            return  # During active trajectory, let the spline drive angles

        alpha = self.spin_alpha.value()
        beta = self.spin_beta.value()

        q_mm, da, R = inverse_kinematics(y=[alpha, beta], z_nominal=Z_HOME)
        self._apply_kinematics_result(q_mm, da, R)

        self.lbl_sb_pose.setText(f"Track: α: {alpha:+.1f}° | β: {beta:.1f}° | Z: {Z_HOME:.1f} mm")

    def _apply_kinematics_result(self, q_mm: np.ndarray, da: np.ndarray, R: np.ndarray):
        """
        Updates 3D visualizer, actuator level bars, Jacobian, transformation matrix,
        and streams serial position command if enabled.
        """
        self.current_q = q_mm
        self.current_da = da
        self.current_R = R

        # 1. Update 3D OpenGL View
        self.gl_view.update_platform(da, R, q_mm)

        # 2. Update Actuator Level Bars
        self.actuator_cluster.update_actuators(q_mm)

        # 3. Calculate Jacobian and Singularity Metrics
        J_inv = InvJac(da, R)
        metrics = calculate_singularity_metrics(J_inv)

        self.lbl_manip_val.setText(f"{metrics['manipulability']:.2e}")
        self.lbl_cond_val.setText(f"{metrics['cond']:.2f}")

        status_text = metrics["status"]
        if metrics["status_code"] == "OPTIMAL":
            self.lbl_singularity_status.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_EMERALD};")
        elif metrics["status_code"] == "SAFE":
            self.lbl_singularity_status.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_CYAN};")
        elif metrics["status_code"] == "WARNING":
            self.lbl_singularity_status.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_WARNING};")
        else:
            self.lbl_singularity_status.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_EMERGENCY};")
        self.lbl_singularity_status.setText(f"ESTADO: {status_text}")

        # 4. Update Transformation Matrix [R | da] Display
        mat_str = (
            f"[ {R[0,0]:+6.3f} {R[0,1]:+6.3f} {R[0,2]:+6.3f} | {da[0]:+6.1f} ]\n"
            f"[ {R[1,0]:+6.3f} {R[1,1]:+6.3f} {R[1,2]:+6.3f} | {da[1]:+6.1f} ]\n"
            f"[ {R[2,0]:+6.3f} {R[2,1]:+6.3f} {R[2,2]:+6.3f} | {da[2]:+6.1f} ]"
        )
        self.txt_matrix.setText(mat_str)

        # 5. Send serial positions if streaming enabled
        if self.chk_stream.isChecked() and self.serial_manager.is_connected:
            self.serial_manager.send_positions(q_mm)

    def _reset_cartesian_zero(self):
        """Zero out XY and rotation, maintain nominal Z."""
        self.spins_pos["X"].setValue(0.0)
        self.spins_pos["Y"].setValue(0.0)
        self.spins_pos["Z"].setValue(Z_HOME)
        self.spins_rot["Roll"].setValue(0.0)
        self.spins_rot["Pitch"].setValue(0.0)
        self.spins_rot["Yaw"].setValue(0.0)

    # =========================================================================
    # TRAJECTORY TRACKING (SimpleCubicSpline Engine)
    # =========================================================================
    def start_trajectory_tracking(self):
        """Generate trajectory spline and start 30 Hz execution timer."""
        pattern = self.combo_traj_pattern.currentText()
        duration = self.spin_traj_dur.value()
        amp_beta = self.spin_traj_amp.value()

        # Build waypoints: t and y=[alpha, beta]
        n_pts = 40
        t_arr = np.linspace(0.0, duration, n_pts)
        y_pts = []

        if "Precesión" in pattern:
            # Cone precession: alpha goes 0 -> 360°, beta constant amplitude
            for t in t_arr:
                alpha = (t / duration) * 360.0 - 180.0
                beta = amp_beta * (0.5 - 0.5 * np.cos(2 * np.pi * t / duration))
                y_pts.append([alpha, beta])

        elif "Figura en 8" in pattern:
            # Polar figure-8 (Lemniscate): alpha oscillates, beta oscillates at 2x frequency
            for t in t_arr:
                tau = 2 * np.pi * (t / duration)
                alpha = 90.0 * np.sin(tau)
                beta = amp_beta * np.abs(np.sin(2 * tau))
                y_pts.append([alpha, beta])

        elif "Espiral" in pattern:
            # Spiral scan: expanding cone
            for t in t_arr:
                fraction = t / duration
                alpha = (fraction * 3.0 * 360.0) % 360.0 - 180.0
                beta = amp_beta * fraction
                y_pts.append([alpha, beta])

        elif "Escalón" in pattern:
            # Step response: smooth transition from 0 to target
            for t in t_arr:
                step_val = amp_beta if t >= duration * 0.25 else 0.0
                y_pts.append([45.0 if t >= duration * 0.25 else 0.0, step_val])

        else:
            # Fallback gentle circle
            for t in t_arr:
                tau = 2 * np.pi * (t / duration)
                y_pts.append([180.0 * np.sin(tau), amp_beta * np.cos(tau)])

        try:
            self.trajectory_spline = SimpleCubicSpline(t=t_arr, y=y_pts)
            self.traj_duration = duration
            self.traj_start_time = time.time()
            self.traj_elapsed = 0.0
            self.is_tracking = True
            self.is_traj_paused = False

            self.btn_start_track.setEnabled(False)
            self.btn_pause_track.setText("⏸ PAUSA TRACK")
            self.traj_progress.setValue(0)
            self.traj_timer.start()

            self.log_message(f"[TRAJ] Seguimiento de trayectoria iniciado: {pattern} ({duration:.1f}s)")
        except Exception as e:
            self.log_error(f"[TRAJ] Error creando Spline: {e}")

    def _trajectory_tick(self):
        """Execute one step of the trajectory spline interpolation."""
        if not self.is_tracking or self.trajectory_spline is None:
            return

        now = time.time()
        self.traj_elapsed = now - self.traj_start_time

        if self.traj_elapsed >= self.traj_duration:
            # Trajectory completed
            self.abort_trajectory()
            self.traj_progress.setValue(100)
            self.log_message("[TRAJ] Trayectoria completada con éxito.")
            return

        # Query cubic spline
        y_interp = self.trajectory_spline.evaluate(self.traj_elapsed)
        alpha, beta = float(y_interp[0]), float(y_interp[1])

        # Reflect in GUI polar controls without retriggering manual event
        self.spin_alpha.blockSignals(True)
        self.spin_beta.blockSignals(True)
        self.slider_alpha.blockSignals(True)
        self.slider_beta.blockSignals(True)

        self.spin_alpha.setValue(alpha)
        self.slider_alpha.setValue(int(alpha))
        self.spin_beta.setValue(beta)
        self.slider_beta.setValue(int(beta * 10))

        self.spin_alpha.blockSignals(False)
        self.spin_beta.blockSignals(False)
        self.slider_alpha.blockSignals(False)
        self.slider_beta.blockSignals(False)

        # Compute and dispatch kinematics
        q_mm, da, R = inverse_kinematics(y=[alpha, beta], z_nominal=Z_HOME)
        self._apply_kinematics_result(q_mm, da, R)

        # Update progress
        pct = int((self.traj_elapsed / self.traj_duration) * 100)
        self.traj_progress.setValue(pct)
        self.lbl_sb_pose.setText(
            f"Tracking t={self.traj_elapsed:.2f}s | α: {alpha:+.1f}° | β: {beta:.1f}°"
        )

    def toggle_pause_trajectory(self):
        """Pause or resume trajectory execution."""
        if not self.is_tracking:
            return

        if not self.is_traj_paused:
            self.traj_timer.stop()
            self.is_traj_paused = True
            self.traj_pause_time = time.time()
            self.btn_pause_track.setText("▶ REANUDAR")
            self.log_message("[TRAJ] Trayectoria en pausa.")
        else:
            pause_delta = time.time() - self.traj_pause_time
            self.traj_start_time += pause_delta
            self.is_traj_paused = False
            self.traj_timer.start()
            self.btn_pause_track.setText("⏸ PAUSA TRACK")
            self.log_message("[TRAJ] Trayectoria reanudada.")

    def abort_trajectory(self):
        """Immediately stop trajectory tracking."""
        self.traj_timer.stop()
        self.is_tracking = False
        self.is_traj_paused = False
        self.btn_start_track.setEnabled(True)
        self.btn_pause_track.setText("⏸ PAUSA TRACK")
        self.traj_progress.setValue(0)
        self.log_message("[TRAJ] Trayectoria detenida.")

    def _load_custom_waypoints(self):
        """Load external CSV file containing (time, alpha, beta) columns."""
        path, _ = QFileDialog.getOpenFileName(self, "Cargar Waypoints CSV", "", "CSV Files (*.csv);;All Files (*)")
        if not path:
            return
        try:
            data = np.loadtxt(path, delimiter=",", skiprows=1)
            if data.shape[1] < 3:
                raise ValueError("El archivo debe contener al menos 3 columnas: tiempo, alpha, beta.")
            t_pts = data[:, 0]
            y_pts = data[:, 1:3]
            self.trajectory_spline = SimpleCubicSpline(t_pts, y_pts)
            self.spin_traj_dur.setValue(float(t_pts[-1] - t_pts[0]))
            self.combo_traj_pattern.setCurrentText("Waypoints Personalizados (CSV)")
            self.log_message(f"[TRAJ] Waypoints cargados exitosamente desde {path} ({len(t_pts)} puntos).")
        except Exception as e:
            QMessageBox.critical(self, "Error de Waypoints", f"No se pudo cargar el archivo:\n{e}")

    # =========================================================================
    # GLOBAL CONTROLS: HOME, PAUSE, ESTOP, CALIBRATION
    # =========================================================================
    def go_home(self):
        """Return platform to nominal home pose."""
        self.abort_trajectory()
        self._reset_cartesian_zero()
        self.serial_manager.home()
        self.log_message("[HOME] Retornando a posición segura nominal.")

    def toggle_pause(self):
        """Global pause."""
        if self.is_tracking:
            self.toggle_pause_trajectory()
        else:
            self.chk_stream.setChecked(not self.chk_stream.isChecked())
            st = "HABILITADO" if self.chk_stream.isChecked() else "PAUSADO"
            self.lbl_sb_stream.setText(f"Streaming: {st}")
            self.log_message(f"[CONTROL] Streaming de actuadores {st}.")

    def trigger_emergency_stop(self):
        """Emergency Stop: immediately halts motion, stops trajectory, disables stream."""
        self.abort_trajectory()
        self.chk_stream.setChecked(False)
        self.lbl_sb_stream.setText("Streaming: ESTOP")
        self.serial_manager.emergency_stop()
        self.badge_safe.set_status("ESTOP ACTIVO", "red")
        self.log_error("[EMERGENCY] !!! PARADA DE EMERGENCIA DISPARADA !!!")

    def calibrate_platform(self):
        """Run automated calibration sequence."""
        self.log_message("[CALIBRATE] Iniciando secuencia de calibración automática...")
        self.serial_manager.calibrate()

    def run_test_pattern(self):
        """Execute demonstration motion pattern."""
        self.tab_widget.setCurrentIndex(1)
        self.start_trajectory_tracking()

    def zero_sensors(self):
        """Zero sensor readouts."""
        self.serial_manager.zero_sensors()
        self.log_message("[SENSORS] Sensores de retroalimentación reseteados a cero.")

    # =========================================================================
    # LOGGING UTILITIES
    # =========================================================================
    @Slot(str)
    def log_message(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.txt_console.append(f"<span style='color: #90A4AE;'>[{timestamp}]</span> {text}")
        # Auto scroll to bottom
        sb = self.txt_console.verticalScrollBar()
        sb.setValue(sb.maximum())

    @Slot(str)
    def log_error(self, text: str):
        timestamp = time.strftime("%H:%M:%S")
        self.txt_console.append(f"<span style='color: #FF5252;'>[{timestamp}] {text}</span>")
        sb = self.txt_console.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _clear_logs(self):
        self.txt_console.clear()

    def closeEvent(self, event):
        """Clean application shutdown."""
        self.abort_trajectory()
        self.serial_manager.close()
        super().closeEvent(event)
