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
    BASE_JOINTS, PLATFORM_JOINTS, Az, Bz, D, RT, ALPHA_0, L0, STROKE,
    T_HOME_END, T_TRACKING_END, DT, FILTER_WN, DEMO_DURATION, TRAJ_POINTS,
    length_to_percent, percent_to_length, q_to_extension, extension_to_q
)
from core.kinematics import (
    pose_to_q, inverse_kinematics, direct_kinematics, home_kinematics,
    InvJac, calculate_singularity_metrics, SimpleCubicSpline
)
from trajectory.generator import TrajectoryConfig, TrajectoryGenerator
from connection.serial_manager import SerialManager
from ui.styles import (
    COLOR_CYAN, COLOR_EMERALD, COLOR_EMERGENCY, COLOR_WARNING,
    COLOR_BORDER, COLOR_CARD_BG, COLOR_PANEL_BG, FONT_MONO
)
from ui.widgets.led_indicator import StatusBadge
from ui.widgets.actuator_bars import ActuatorClusterWidget
from ui.widgets.gl_view import Platform3DView
from ui.widgets.telemetry_plot import TelemetryPlotWidget
from ui.dialogs import ParametersDialog, SerialConfigDialog, load_parameters
import config.parameters as param_module


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

        # Synchronize runtime parameters from config/parameters.json
        self._sync_runtime_parameters(load_parameters())

        # Initialize Serial Communication Manager
        self.serial_manager = SerialManager(self)

        # State Variables initialized at HOME (all actuators retracted = 0.0 mm extension)
        q_home, da_home, R_home = home_kinematics()
        self.current_da = da_home.copy()
        self.current_R = R_home.copy()
        self.current_q = q_home.copy()
        self.current_pct = np.zeros(6, dtype=np.float64)
        self.actual_q = q_home.copy()
        self.control_mode = "CARTESIAN"  # 'CARTESIAN', 'TRACK', or 'IDLE'

        # Trajectory Tracking Engine using authentic TrajectoryGenerator
        self.traj_cfg = TrajectoryConfig(
            t_home_end=param_module.T_HOME_END,
            t_tracking_end=param_module.T_TRACKING_END,
            dt=param_module.DT,
            filter_wn=param_module.FILTER_WN
        )
        self.traj_gen = TrajectoryGenerator(self.traj_cfg)
        self.current_traj_points = [list(pt) for pt in param_module.TRAJ_POINTS]
        self.traj_gen.set_tracking_points(np.radians(self.current_traj_points))

        self.traj_timer = QTimer(self)
        self.traj_timer.setInterval(33)  # ~30 Hz execution loop
        self.traj_start_time = 0.0
        self.traj_pause_time = 0.0
        self.traj_elapsed = 0.0
        self.traj_duration = param_module.DEMO_DURATION
        self.is_tracking = False
        self.is_traj_paused = False
        self.hold_phase = None
        self.last_tick_time = 0.0
        self.custom_traj_points = None
        self.csv_trajectory = None
        self.is_csv_playback = False

        # Build UI Structure
        self._setup_ui()
        self._connect_signals()

        # Initialize platform at HOME (all actuators retracted at 0.0 mm)
        self.go_home()

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
        self.btn_connect.setFixedWidth(100)
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        # Serial Config Dialog Button
        self.btn_serial_config = QPushButton("⚙ CONFIG")
        self.btn_serial_config.setFixedWidth(75)
        self.btn_serial_config.setToolTip("Configuración de comunicación serial y simulador")
        self.btn_serial_config.clicked.connect(self.open_serial_config)
        layout.addWidget(self.btn_serial_config)

        layout.addSpacing(10)

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

        # Parameters Dialog Button
        self.btn_parameters = QPushButton("📐 PARÁMETROS")
        self.btn_parameters.setToolTip("Configuración de parámetros del sistema (Geometría, Actuadores, Trayectoria, Az/Bz)")
        self.btn_parameters.clicked.connect(self.open_parameter_dialog)
        layout.addWidget(self.btn_parameters)

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
            ("Z", LIMITS_Z[0], LIMITS_Z[1], 0.0, "mm")
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

        # Pointing Angle Controls: Alpha (Azimuth) & Beta (Elevation from Horizon)
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

        # Beta (Elevation from horizon, 90° = Cenit/Home)
        row_beta = QHBoxLayout()
        lbl_b = QLabel("Beta (β):")
        lbl_b.setStyleSheet("font-weight: 700; color: #00E5FF;")
        lbl_b.setFixedWidth(70)

        self.slider_beta = QSlider(Qt.Orientation.Horizontal)
        self.slider_beta.setRange(int(LIMITS_BETA[0] * 10), int(LIMITS_BETA[1] * 10))
        self.slider_beta.setValue(900)  # Cenit = 90°

        self.spin_beta = QDoubleSpinBox()
        self.spin_beta.setRange(LIMITS_BETA[0], LIMITS_BETA[1])
        self.spin_beta.setDecimals(1)
        self.spin_beta.setValue(90.0)
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
        grp_traj = QGroupBox("Generador de Trayectorias (Secuencia APPROACH → TRACK → RETURN)")
        traj_layout = QVBoxLayout(grp_traj)
        traj_layout.setSpacing(8)

        # Profile Selector
        row_prof = QHBoxLayout()
        lbl_prof = QLabel("Perfil:")
        lbl_prof.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.combo_traj_pattern = QComboBox()
        self.combo_traj_pattern.addItems([
            "Secuencia Demo (LMT: 10.1°, 45.6°, 10.0°)",
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
        self.spin_traj_dur.setRange(3.0, 720.0)
        self.spin_traj_dur.setValue(60.0)
        self.spin_traj_dur.setSuffix(" s")
        self.spin_traj_dur.setFixedWidth(75)

        lbl_amp = QLabel("Amplitud β:")
        lbl_amp.setStyleSheet("color: #90A4AE; font-size: 11px;")
        self.spin_traj_amp = QDoubleSpinBox()
        self.spin_traj_amp.setRange(2.0, 80.0)
        self.spin_traj_amp.setValue(45.0)
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

        self.btn_params_bottom = QPushButton("📐 PARÁMETROS")
        self.btn_params_bottom.clicked.connect(self.open_parameter_dialog)
        btn_bar.addWidget(self.btn_params_bottom)

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
        vals = np.array(actual_vals, dtype=np.float64)
        if len(vals) == 6:
            if np.max(vals) <= 100.0:
                self.actual_q = percent_to_length(vals, param_module.L0, param_module.STROKE)
            else:
                self.actual_q = vals
            # Update telemetry plot with target vs actual
            self.telemetry_plot.update_telemetry(self.current_q, self.actual_q)

    # =========================================================================
    # KINEMATICS & CONTROL DISPATCH
    # =========================================================================
    def _on_tab_changed(self, index: int):
        if index == 0:
            self.control_mode = "CARTESIAN"
            self.badge_mode.set_status("CARTESIANO", "cyan")
            self._on_cartesian_changed()
        elif index == 1:
            self.control_mode = "TRACK"
            self.badge_mode.set_status("TRACK (α/β)", "cyan")
            self._on_track_angles_changed()
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

        # Compute Inverse Kinematics (degrees converted automatically)
        q_mm, da, R = pose_to_q(x, y, z, roll, pitch, yaw, deg=True)
        self._apply_kinematics_result(q_mm, da, R)

        self.lbl_sb_pose.setText(
            f"Pose: X:{x:+.1f} Y:{y:+.1f} Z:{z:.1f} | R:{roll:+.1f}° P:{pitch:+.1f}° Y:{yaw:+.1f}°"
        )

    def _on_track_angles_changed(self):
        """Called when Alpha or Beta slider / spinbox changes (hemisphere constraint active)."""
        if self.is_tracking:
            return  # During active trajectory, let TrajectoryGenerator drive angles

        alpha = self.spin_alpha.value()
        beta = float(np.clip(self.spin_beta.value(), 0.0, 90.0))

        # Compute Inverse Kinematics on the hemisphere (da = D + rt * R[:, 2], beta in [0°, 90°])
        q_mm, da, R = inverse_kinematics(
            [alpha, beta],
            rt=param_module.RT,
            a0=param_module.ALPHA_0,
            D=param_module.D,
            deg=True,
            clamp_hemisphere=True
        )
        self._apply_kinematics_result(q_mm, da, R)

        self.lbl_sb_pose.setText(
            f"Track (Semiesfera rt={param_module.RT:.0f}mm): α: {alpha:+.1f}° | β: {beta:.1f}° | Z: {da[2]:.1f} mm"
        )

    def _apply_kinematics_result(self, q_mm: np.ndarray, da: np.ndarray, R: np.ndarray):
        """
        Updates 3D visualizer, actuator level bars, Jacobian, transformation matrix,
        telemetry plot, and streams serial position command if enabled.
        """
        self.current_q = q_mm
        self.current_da = da
        self.current_R = R
        if not self.serial_manager.is_connected:
            self.actual_q = q_mm.copy()

        # 1. Update 3D OpenGL View
        self.gl_view.update_platform(da, R, q_mm)

        # 2. Update Actuator Level Bars (shows extension q_ef = q_mm - L0 in mm; HOME = 0.0 mm)
        self.actuator_cluster.update_actuators(q_mm)

        # 3. Calculate Jacobian and Singularity Metrics
        J_inv = InvJac(da, R, Az=param_module.Az, Bz=param_module.Bz)
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

        # 5. Convert to stroke percent and stream serial command if enabled
        pct = length_to_percent(q_mm, l0=param_module.L0, stroke=param_module.STROKE)
        self.current_pct = pct

        if self.chk_stream.isChecked() and self.serial_manager.is_connected:
            self.serial_manager.send_positions(pct)

        # 6. Update telemetry plot with target vs actual
        self.telemetry_plot.update_telemetry(self.current_q, self.actual_q)

    def _reset_cartesian_zero(self):
        """Zero out XY and rotation, and set nominal relative Z offset to 0.0."""
        for axis in ["X", "Y", "Z"]:
            if axis in self.spins_pos:
                self.spins_pos[axis].blockSignals(True)
                self.spins_pos[axis].setValue(0.0)
                if axis in self.sliders_pos:
                    self.sliders_pos[axis].setValue(0)
                self.spins_pos[axis].blockSignals(False)

        for rot in ["Roll", "Pitch", "Yaw"]:
            if rot in self.spins_rot:
                self.spins_rot[rot].blockSignals(True)
                self.spins_rot[rot].setValue(0.0)
                if rot in self.sliders_rot:
                    self.sliders_rot[rot].setValue(0)
                self.spins_rot[rot].blockSignals(False)

    # =========================================================================
    # TRAJECTORY TRACKING (4-Phase Sequence & MATLAB CSV Playback)
    # =========================================================================
    def start_trajectory_tracking(self):
        """
        Execute trajectory sequence:
        - If 'Waypoints Personalizados (CSV)' is selected and a CSV trajectory is loaded:
          reproduces the smoothly interpolated CSV sequence (either alpha/beta via Inverse Kinematics
          or L1..L6 directly).
        - Otherwise: executes the 4-phase sequence HOME (0 mm) → APPROACH → TRACK → RETURN → HOME (0 mm).
        """
        pattern = self.combo_traj_pattern.currentText()
        duration = self.spin_traj_dur.value()
        amp_beta = self.spin_traj_amp.value()

        # Case 1: Continuous MATLAB CSV Trajectory Playback
        if "Waypoints Personalizados" in pattern and self.csv_trajectory is not None:
            try:
                csv_dur = float(self.csv_trajectory["duration"])
                self.traj_duration = float(duration) if duration > 0.0 else max(csv_dur, 1.0)
                self.traj_elapsed = 0.0
                self.last_tick_time = time.time()
                self.is_tracking = True
                self.is_csv_playback = True
                self.is_traj_paused = False
                self.hold_phase = "CSV"

                self.btn_start_track.setEnabled(False)
                self.btn_pause_track.setText("⏸ PAUSA TRACK")
                self.traj_progress.setValue(0)
                self.badge_mode.set_status("TRAJ: CSV", "cyan")
                self.traj_timer.start()

                mode_label = (
                    "α/β (Cinemática Inversa)"
                    if self.csv_trajectory["mode"] == "alpha_beta"
                    else "L1–L6 Directo"
                )
                self.log_message(
                    f"[CSV] Reproducción iniciada ({mode_label}): "
                    f"{self.csv_trajectory['num_points']} puntos originales, "
                    f"duración={self.traj_duration:.2f}s."
                )
                return
            except Exception as e:
                self.log_error(f"[CSV] Error iniciando reproducción CSV: {e}")
                return

        # Case 2: Standard 4-phase TrajectoryGenerator sequence
        self.is_csv_playback = False

        # Partition duration: home/approach (min 10s or 16%), track, return
        t_home = min(10.0, max(2.0, duration * (10.0 / 60.0)))
        t_track = max(t_home + 2.0, duration - t_home)

        # Select 3 waypoints [alpha, beta] in degrees (beta = elevation from horizon in [0°, 90°])
        if "Secuencia Demo" in pattern:
            pts_deg = [list(pt) for pt in self.current_traj_points]
        elif "Precesión" in pattern:
            elev = max(5.0, 90.0 - amp_beta)
            pts_deg = [[0.0, elev], [180.0, elev], [360.0, elev]]
        elif "Figura en 8" in pattern:
            elev_min = max(5.0, 90.0 - amp_beta)
            pts_deg = [[-45.0, elev_min], [0.0, 90.0 - amp_beta * 0.3], [45.0, elev_min]]
        elif "Espiral" in pattern:
            pts_deg = [[30.0, 90.0 - amp_beta * 0.2], [180.0, 90.0 - amp_beta * 0.6], [360.0, 90.0 - amp_beta]]
        elif "Escalón" in pattern:
            elev = max(5.0, 90.0 - amp_beta)
            pts_deg = [[45.0, elev], [45.0, elev], [45.0, elev]]
        elif "Waypoints Personalizados" in pattern and self.custom_traj_points is not None:
            pts_deg = [list(pt) for pt in self.custom_traj_points]
        else:
            pts_deg = [list(pt) for pt in self.current_traj_points]

        try:
            pts_rad = np.radians(pts_deg)
            self.traj_cfg = TrajectoryConfig(
                t_home_end=t_home,
                t_tracking_end=t_track,
                dt=0.033,
                filter_wn=param_module.FILTER_WN
            )
            self.traj_gen = TrajectoryGenerator(self.traj_cfg)
            self.traj_gen.set_tracking_points(pts_rad)
            self.traj_gen.reset()

            self.traj_duration = t_track + t_home
            self.traj_elapsed = 0.0
            self.last_tick_time = time.time()
            self.is_tracking = True
            self.is_traj_paused = False
            self.hold_phase = None

            self.btn_start_track.setEnabled(False)
            self.btn_pause_track.setText("⏸ PAUSA TRACK")
            self.traj_progress.setValue(0)
            self.badge_mode.set_status("TRAJ: APPROACH", "cyan")
            self.traj_timer.start()

            self.log_message(
                f"[TRAJ] Secuencia iniciada desde HOME (0 mm): {pattern} (Total: {self.traj_duration:.1f}s, "
                f"Approach: {t_home:.1f}s, Track: {t_track - t_home:.1f}s, Return: {t_home:.1f}s, rt={param_module.RT:.1f}mm)"
            )
        except Exception as e:
            self.log_error(f"[TRAJ] Error iniciando generador de trayectoria: {e}")

    def _eval_csv_trajectory(self, t_csv: float) -> np.ndarray:
        """Evaluate smoothly interpolated CSV trajectory at time t_csv."""
        traj = self.csv_trajectory
        t_clamped = float(np.clip(t_csv, 0.0, traj["duration"]))
        if traj["spline"] is not None:
            return np.asarray(traj["spline"](t_clamped), dtype=np.float64)
        # Dense linear interpolation fallback
        t_arr = traj["t_dense"]
        y_arr = traj["y_dense"]
        cols = y_arr.shape[1]
        out = np.zeros(cols, dtype=np.float64)
        for c in range(cols):
            out[c] = float(np.interp(t_clamped, t_arr, y_arr[:, c]))
        return out

    def _apply_csv_sample(self, t_csv: float):
        """Evaluate and apply one interpolated CSV sample to kinematics, 3D view, and telemetry."""
        vals = self._eval_csv_trajectory(t_csv)
        mode = self.csv_trajectory["mode"]

        if mode == "alpha_beta":
            alpha_deg = float(vals[0])
            beta_deg = float(np.clip(vals[1], 0.0, 90.0))

            self.spin_alpha.blockSignals(True)
            self.spin_beta.blockSignals(True)
            self.slider_alpha.blockSignals(True)
            self.slider_beta.blockSignals(True)

            self.spin_alpha.setValue(alpha_deg)
            self.slider_alpha.setValue(int(round(alpha_deg)))
            self.spin_beta.setValue(beta_deg)
            self.slider_beta.setValue(int(round(beta_deg * 10.0)))

            self.spin_alpha.blockSignals(False)
            self.spin_beta.blockSignals(False)
            self.slider_alpha.blockSignals(False)
            self.slider_beta.blockSignals(False)

            q_mm, da, R = inverse_kinematics(
                [alpha_deg, beta_deg],
                rt=param_module.RT,
                a0=param_module.ALPHA_0,
                D=param_module.D,
                deg=True,
                clamp_hemisphere=True
            )
            self._apply_kinematics_result(q_mm, da, R)
            return alpha_deg, beta_deg, da, q_mm
        else:
            # Direct actuator lengths L1..L6 (already in total anchor-to-anchor mm)
            q_mm = np.asarray(vals[:6], dtype=np.float64)
            try:
                Y_deg, da, R = direct_kinematics(
                    q_mm,
                    da0=self.current_da,
                    R0=self.current_R,
                    Az=param_module.Az,
                    Bz=param_module.Bz,
                    D=param_module.D,
                    rt=param_module.RT,
                    a0=param_module.ALPHA_0,
                    deg=True
                )
                alpha_deg = float(Y_deg[0])
                beta_deg = float(np.clip(Y_deg[1], 0.0, 90.0))
            except Exception:
                da = self.current_da.copy()
                R = self.current_R.copy()
                alpha_deg = float(self.spin_alpha.value())
                beta_deg = float(self.spin_beta.value())

            self.spin_alpha.blockSignals(True)
            self.spin_beta.blockSignals(True)
            self.slider_alpha.blockSignals(True)
            self.slider_beta.blockSignals(True)

            self.spin_alpha.setValue(alpha_deg)
            self.slider_alpha.setValue(int(round(alpha_deg)))
            self.spin_beta.setValue(beta_deg)
            self.slider_beta.setValue(int(round(beta_deg * 10.0)))

            self.spin_alpha.blockSignals(False)
            self.spin_beta.blockSignals(False)
            self.slider_alpha.blockSignals(False)
            self.slider_beta.blockSignals(False)

            self._apply_kinematics_result(q_mm, da, R)
            return alpha_deg, beta_deg, da, q_mm

    def _trajectory_tick(self):
        """Execute one step of the active trajectory tracking sequence (CSV or 4-phase generator)."""
        if not self.is_tracking:
            return

        now = time.time()
        dt = now - self.last_tick_time
        self.last_tick_time = now
        if dt <= 0.0 or dt > 0.15:
            dt = 0.033

        self.traj_elapsed += dt
        t = self.traj_elapsed

        # Branch A: Smooth MATLAB CSV trajectory playback
        if self.is_csv_playback and self.csv_trajectory is not None:
            total_dur = max(float(self.traj_duration), 1e-6)
            csv_dur = float(self.csv_trajectory["duration"])

            if t >= total_dur:
                # Apply final sample and finish cleanly
                alpha_deg, beta_deg, da, _ = self._apply_csv_sample(csv_dur)
                self.traj_timer.stop()
                self.is_tracking = False
                self.is_csv_playback = False
                self.is_traj_paused = False
                self.btn_start_track.setEnabled(True)
                self.btn_pause_track.setText("⏸ PAUSA TRACK")
                self.traj_progress.setValue(100)
                self.badge_mode.set_status("CSV COMPLETADO", "green")
                self.lbl_sb_pose.setText(
                    f"[CSV FIN] t={total_dur:.2f}s / {total_dur:.2f}s | α: {alpha_deg:+.1f}° | β: {beta_deg:.1f}° | Z: {da[2]:.1f} mm"
                )
                self.log_message("[CSV] Reproducción de trayectoria CSV completada con éxito.")
                return

            t_csv = (t / total_dur) * csv_dur if csv_dur > 0.0 else 0.0
            alpha_deg, beta_deg, da, _ = self._apply_csv_sample(t_csv)
            pct = int(min(100, max(0, round((t / total_dur) * 100.0))))
            self.traj_progress.setValue(pct)
            self.badge_mode.set_status("TRAJ: CSV", "cyan")
            self.lbl_sb_pose.setText(
                f"[CSV] t={t:.2f}s / {total_dur:.1f}s | α: {alpha_deg:+.1f}° | β: {beta_deg:.1f}° | Z: {da[2]:.1f} mm"
            )
            return

        # Branch B: Standard 4-phase TrajectoryGenerator
        if self.traj_gen is None:
            return

        t_home_end = self.traj_cfg.t_home_end
        t_track_end = self.traj_cfg.t_tracking_end
        t_return_end = t_track_end + t_home_end

        if t >= t_return_end or self.traj_gen.phase == "done":
            # Sequence complete: transition to HOME (0 mm retracted)
            self.go_home()
            self.traj_progress.setValue(100)
            self.log_message("[TRAJ] Secuencia completada con éxito: RETURN → HOME (actuadores en 0.0 mm).")
            return

        # Query authentic TrajectoryGenerator
        y_des, yp_des = self.traj_gen.step(t, dt=dt)
        alpha_rad = float(y_des[0])
        beta_rad = float(np.clip(y_des[1], 0.0, np.pi / 2.0))
        alpha_deg = float(np.rad2deg(alpha_rad))
        beta_deg = float(np.rad2deg(beta_rad))

        # Smooth transition between retracted HOME (rt_eff = 0 mm) and Hemisphere (rt_eff = RT)
        rt_full = float(param_module.RT)
        if self.traj_gen.phase == "home":
            tau = float(np.clip(t / max(t_home_end, 1e-6), 0.0, 1.0))
            s = tau * tau * (3.0 - 2.0 * tau)
            rt_eff = s * rt_full
        elif self.traj_gen.phase == "return":
            tau = float(np.clip((t - t_track_end) / max(t_home_end, 1e-6), 0.0, 1.0))
            s = tau * tau * (3.0 - 2.0 * tau)
            rt_eff = (1.0 - s) * rt_full
        else:
            rt_eff = rt_full

        # Phase mapping: "home" -> APPROACH, "tracking" -> TRACK, "return" -> RETURN, "done" -> HOME
        phase_map = {"home": "APPROACH", "tracking": "TRACK", "return": "RETURN", "done": "HOME"}
        phase_str = phase_map.get(self.traj_gen.phase, "TRACK")
        self.badge_mode.set_status(f"TRAJ: {phase_str}", "cyan")

        # Update GUI sliders / spinboxes without retriggering manual event
        self.spin_alpha.blockSignals(True)
        self.spin_beta.blockSignals(True)
        self.slider_alpha.blockSignals(True)
        self.slider_beta.blockSignals(True)

        self.spin_alpha.setValue(alpha_deg)
        self.slider_alpha.setValue(int(alpha_deg))
        self.spin_beta.setValue(beta_deg)
        self.slider_beta.setValue(int(beta_deg * 10))

        self.spin_alpha.blockSignals(False)
        self.spin_beta.blockSignals(False)
        self.slider_alpha.blockSignals(False)
        self.slider_beta.blockSignals(False)

        # Compute and dispatch kinematics on the hemisphere
        q_mm, da, R = inverse_kinematics(
            [alpha_rad, beta_rad],
            rt=rt_eff,
            a0=param_module.ALPHA_0,
            D=param_module.D,
            deg=False,
            clamp_hemisphere=True
        )
        self._apply_kinematics_result(q_mm, da, R)

        # Update progress bar
        pct = int(min(100, max(0, (t / t_return_end) * 100)))
        self.traj_progress.setValue(pct)
        self.lbl_sb_pose.setText(
            f"[{phase_str}] t={t:.2f}s / {t_return_end:.1f}s | α: {alpha_deg:+.1f}° | β: {beta_deg:.1f}° | Z: {da[2]:.1f} mm"
        )

    def toggle_pause_trajectory(self):
        """Pause or resume trajectory execution in HOLD state."""
        if not self.is_tracking:
            return

        if not self.is_traj_paused:
            self.traj_timer.stop()
            self.is_traj_paused = True
            if self.is_csv_playback:
                self.hold_phase = "CSV"
            else:
                self.hold_phase = self.traj_gen.phase if self.traj_gen else "HOLD"
            self.btn_pause_track.setText("▶ REANUDAR")
            self.badge_mode.set_status(f"HOLD ({self.hold_phase.upper()})", "amber")
            self.log_message(f"[TRAJ] Pausa activada: plataforma en HOLD ({self.hold_phase.upper()}).")
        else:
            self.is_traj_paused = False
            self.last_tick_time = time.time()
            self.traj_timer.start()
            self.btn_pause_track.setText("⏸ PAUSA TRACK")
            phase_str = self.hold_phase.upper() if self.hold_phase else "TRACK"
            self.badge_mode.set_status(f"TRAJ: {phase_str}", "cyan")
            self.log_message("[TRAJ] Trayectoria reanudada.")

    def abort_trajectory(self):
        """Immediately stop trajectory tracking."""
        self.traj_timer.stop()
        self.is_tracking = False
        self.is_csv_playback = False
        self.is_traj_paused = False
        self.btn_start_track.setEnabled(True)
        self.btn_pause_track.setText("⏸ PAUSA TRACK")
        self.traj_progress.setValue(0)
        self.badge_mode.set_status("TRACK (α/β)", "cyan")
        self.log_message("[TRAJ] Trayectoria detenida.")

    def _parse_csv_file(self, path: str) -> dict:
        """
        Parse and smoothly interpolate a MATLAB-exported CSV trajectory file.
        Supported formats:
          1) time, alpha, beta                 (3 columns, angles in degrees)
             or alpha, beta                    (2 columns, angles in degrees)
          2) time, L1, L2, L3, L4, L5, L6      (7 columns, actuator lengths in mm or %)
             or L1, L2, L3, L4, L5, L6         (6 columns, actuator lengths in mm or %)
        """
        import re as _re

        with open(path, "r", encoding="utf-8-sig") as fh:
            raw_lines = [ln.strip() for ln in fh.readlines() if ln.strip() and not ln.strip().startswith(("#", "%"))]

        if not raw_lines:
            raise ValueError("El archivo CSV está vacío.")

        header_text = ""
        numeric_rows = []
        for idx, ln in enumerate(raw_lines):
            parts = [p.strip() for p in _re.split(r"[,;\t]+|\s+", ln) if p.strip()]
            try:
                row_vals = [float(p) for p in parts]
                numeric_rows.append(row_vals)
            except ValueError:
                if idx == 0:
                    header_text = ln.lower()
                else:
                    continue

        if not numeric_rows:
            raise ValueError("No se encontraron filas numéricas válidas en el CSV.")

        data = np.asarray(numeric_rows, dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)

        ncols = data.shape[1]
        default_dur = float(self.spin_traj_dur.value()) if hasattr(self, "spin_traj_dur") else 60.0

        if ncols == 3:
            t_raw = data[:, 0]
            y_raw = data[:, 1:3]
            mode = "alpha_beta"
        elif ncols == 2:
            t_raw = np.linspace(0.0, default_dur, len(data))
            y_raw = data[:, 0:2]
            mode = "alpha_beta"
        elif ncols >= 7:
            t_raw = data[:, 0]
            L_raw = data[:, 1:7]
            mode = "lengths"
        elif ncols == 6:
            t_raw = np.linspace(0.0, default_dur, len(data))
            L_raw = data[:, 0:6]
            mode = "lengths"
        else:
            raise ValueError(
                f"Formato CSV no soportado ({ncols} columnas). "
                "Formatos esperados: [time, alpha, beta] (3 cols) o [time, L1..L6] (7 cols)."
            )

        if mode == "lengths":
            # Convert L1..L6 to total anchor-to-anchor length q_mm (mm):
            # - If values are in [0, STROKE+40] (extension in mm or % stroke), convert to total length L0 + ext
            # - If values are > STROKE+40 (already total anchor-to-anchor length in mm), use directly
            if "%" in header_text or "pct" in header_text or "percent" in header_text:
                y_raw = percent_to_length(L_raw, param_module.L0, param_module.STROKE)
            elif np.max(L_raw) <= param_module.STROKE + 40.0:
                y_raw = extension_to_q(L_raw, param_module.L0)
            else:
                y_raw = L_raw.copy()

        # Normalize time vector to start at 0.0s and ensure strictly increasing timestamps
        t_norm = np.asarray(t_raw, dtype=np.float64) - float(t_raw[0])
        order = np.argsort(t_norm, kind="mergesort")
        t_norm = t_norm[order]
        y_raw = y_raw[order]

        if len(t_norm) > 1:
            unique_mask = np.concatenate([[True], np.diff(t_norm) > 1e-7])
            t_norm = t_norm[unique_mask]
            y_raw = y_raw[unique_mask]

        if len(t_norm) == 1 or float(t_norm[-1]) <= 0.0:
            t_norm = np.linspace(0.0, max(default_dur, 1.0), len(y_raw))

        duration = float(t_norm[-1])

        # Build smooth cubic spline interpolator (or dense linear table if < 3 points)
        spline_obj = None
        if len(t_norm) >= 3:
            try:
                spline_obj = SimpleCubicSpline(t_norm, y_raw, bc_type="natural")
            except Exception:
                spline_obj = None

        n_dense = int(min(20000, max(1000, len(t_norm) * 10)))
        t_dense = np.linspace(0.0, duration, n_dense)
        if spline_obj is not None:
            y_dense = np.asarray(spline_obj(t_dense), dtype=np.float64)
        else:
            y_dense = np.zeros((n_dense, y_raw.shape[1]), dtype=np.float64)
            for c in range(y_raw.shape[1]):
                y_dense[:, c] = np.interp(t_dense, t_norm, y_raw[:, c])

        return {
            "path": path,
            "mode": mode,
            "t": t_norm,
            "y": y_raw,
            "t_dense": t_dense,
            "y_dense": y_dense,
            "spline": spline_obj,
            "duration": duration,
            "num_points": len(data),
        }

    def _load_custom_waypoints(self, path: Optional[str] = None):
        """
        Load external MATLAB CSV file containing either:
          - time, alpha, beta (in degrees)
          - time, L1, L2, L3, L4, L5, L6 (lengths in mm or %)
        Interpolates smoothly and prepares continuous trajectory playback.
        """
        if not path:
            path, _ = QFileDialog.getOpenFileName(
                self, "Cargar Trayectoria CSV (MATLAB)", "", "CSV Files (*.csv);;All Files (*)"
            )
        if not path:
            return
        try:
            csv_traj = self._parse_csv_file(path)
            self.csv_trajectory = csv_traj

            # Also keep 3 representative waypoints for compatibility if needed
            if csv_traj["mode"] == "alpha_beta":
                pts = csv_traj["y"]
                if len(pts) >= 3:
                    indices = [0, len(pts) // 2, -1]
                    self.custom_traj_points = [pts[i].tolist() for i in indices]
                elif len(pts) == 1:
                    self.custom_traj_points = [pts[0].tolist()] * 3
                else:
                    self.custom_traj_points = [pts[0].tolist(), pts[1].tolist(), pts[1].tolist()]

            self.combo_traj_pattern.setCurrentText("Waypoints Personalizados (CSV)")

            # Sync spin_traj_dur with CSV duration if positive
            if csv_traj["duration"] > 0.0:
                clamped_dur = float(
                    np.clip(csv_traj["duration"], self.spin_traj_dur.minimum(), self.spin_traj_dur.maximum())
                )
                self.spin_traj_dur.setValue(clamped_dur)

            # Preview initial point (t = 0) on the 3D view and telemetry
            self._apply_csv_sample(0.0)
            self.traj_progress.setValue(0)

            mode_desc = "time, alpha, beta (°)" if csv_traj["mode"] == "alpha_beta" else "time, L1..L6 (mm/%)"
            self.log_message(
                f"[CSV] Trayectoria cargada e interpolada desde '{path}': "
                f"formato [{mode_desc}], {csv_traj['num_points']} puntos, "
                f"duración={csv_traj['duration']:.2f}s. Presione '▶ INICIAR TRACKING' para reproducir."
            )
        except Exception as e:
            QMessageBox.critical(self, "Error Cargando CSV", f"No se pudo cargar el archivo CSV:\n{e}")

    # =========================================================================
    # GLOBAL CONTROLS: HOME, PAUSE, ESTOP, CALIBRATION
    # =========================================================================
    def go_home(self):
        """Return platform to nominal home pose (Zenith: alpha=0, beta=90°, all actuators retracted at 0.0 mm / 0%)."""
        self.abort_trajectory()
        self._reset_cartesian_zero()

        # Reset polar inputs to Zenith (alpha=0, beta=90°)
        self.spin_alpha.blockSignals(True)
        self.spin_beta.blockSignals(True)
        self.slider_alpha.blockSignals(True)
        self.slider_beta.blockSignals(True)

        self.spin_alpha.setValue(0.0)
        self.slider_alpha.setValue(0)
        self.spin_beta.setValue(90.0)
        self.slider_beta.setValue(900)

        self.spin_alpha.blockSignals(False)
        self.spin_beta.blockSignals(False)
        self.slider_alpha.blockSignals(False)
        self.slider_beta.blockSignals(False)

        if hasattr(self, 'traj_gen') and self.traj_gen:
            self.traj_gen.reset()

        # Compute retracted HOME kinematics: da = D, R = I_3, q_mm = L0 => extension q_ef = 0.0 mm (0%)
        q_mm, da, R = home_kinematics(D=param_module.D, Az=param_module.Az, Bz=param_module.Bz)
        self._apply_kinematics_result(q_mm, da, R)

        # Send home command via serial
        self.serial_manager.home()
        self.badge_mode.set_status("HOME (0 mm)", "cyan")
        self.lbl_sb_pose.setText(
            f"HOME (Retraído 0.0 mm / 0%): α: 0.0° | β: 90.0° | Z: {da[2]:.2f} mm"
        )
        self.log_message("[HOME] Posición HOME alcanzada: todos los actuadores retraídos en 0.0 mm (0%).")

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

    def open_serial_config(self):
        """Open modal serial communication configuration dialog."""
        dialog = SerialConfigDialog(self.serial_manager, self)
        dialog.exec()
        self.refresh_ports()

    def open_parameter_dialog(self):
        """Open modal system parameters dialog."""
        dialog = ParametersDialog(current_params=load_parameters(), parent=self)
        dialog.parameters_changed.connect(self._on_parameters_changed)
        dialog.exec()

    def _sync_runtime_parameters(self, params: dict):
        """Synchronize config.parameters module attributes with loaded parameter dictionary."""
        if not isinstance(params, dict):
            return
        if "L0" in params:
            param_module.L0 = float(params["L0"])
        if "STROKE" in params:
            param_module.STROKE = float(params["STROKE"])
        param_module.ACTUATOR_MIN = param_module.L0
        param_module.ACTUATOR_MAX = param_module.L0 + param_module.STROKE
        param_module.Q_MIN = param_module.ACTUATOR_MIN
        param_module.Q_MAX = param_module.ACTUATOR_MAX
        param_module.Q_NOMINAL = param_module.ACTUATOR_MIN
        param_module.Q_SAFE_MIN = param_module.Q_MIN + param_module.SAFE_MARGIN
        param_module.Q_SAFE_MAX = param_module.Q_MAX - param_module.SAFE_MARGIN

        if "RT" in params:
            param_module.RT = float(params["RT"])
        if "ALPHA_0" in params:
            param_module.ALPHA_0 = float(params["ALPHA_0"])
        if "D" in params:
            param_module.D = np.array(params["D"], dtype=np.float64)
            param_module.Z_HOME = float(param_module.D[2])
        if "Az" in params:
            param_module.Az = np.array(params["Az"], dtype=np.float64)
            param_module.PLATFORM_JOINTS = param_module.Az
        if "Bz" in params:
            param_module.Bz = np.array(params["Bz"], dtype=np.float64)
            param_module.BASE_JOINTS = param_module.Bz
        if "ALPHA_PI" in params:
            param_module.ALPHA_PI = float(params["ALPHA_PI"])
        if "KP_S" in params:
            param_module.KP_S = float(params["KP_S"])
        if "KI_S" in params:
            param_module.KI_S = float(params["KI_S"])
        if "T_HOME_END" in params:
            param_module.T_HOME_END = float(params["T_HOME_END"])
        if "T_TRACKING_END" in params:
            param_module.T_TRACKING_END = float(params["T_TRACKING_END"])
        if "DEMO_DURATION" in params:
            param_module.DEMO_DURATION = float(params["DEMO_DURATION"])
        if "DT" in params:
            param_module.DT = float(params["DT"])
        if "FILTER_WN" in params:
            param_module.FILTER_WN = float(params["FILTER_WN"])
        if "TRAJ_POINTS" in params:
            param_module.TRAJ_POINTS = [list(pt) for pt in params["TRAJ_POINTS"]]

    def _on_parameters_changed(self, params: dict):
        """Handle system parameters update and refresh kinematics/views."""
        self.log_message("[CONFIG] Nuevos parámetros del sistema aplicados y guardados.")
        self._sync_runtime_parameters(params)
        self.current_da = param_module.D.copy()
        self.gl_view.platform_joints = param_module.Az.copy()
        self.gl_view.base_joints = param_module.Bz.copy()
        self.current_traj_points = [list(pt) for pt in param_module.TRAJ_POINTS]

        # Re-evaluate kinematics to reflect updated geometry
        if self.control_mode == "CARTESIAN":
            self._on_cartesian_changed()
        else:
            self._on_track_angles_changed()

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
