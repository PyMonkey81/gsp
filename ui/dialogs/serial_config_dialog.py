"""
SerialConfigDialog (Actuonix LAC 6-Port USB Hub Configuration Dialog)
for Gough-Stewart 6-DOF Platform.
Provides:
  - USB Hub discovery & connection for 6 Actuonix LAC boards (or Virtual Simulator)
  - Live per-board status and Serial ID assignment (0x50 SET_SERIAL_NUM / 0x51 GET_SERIAL_NUM)
  - Full Advanced LAC Configuration parameters (0x01..0x0F, 0x30 Disable Manual EEPROM, 0xFF Reset)
  - Closed-loop PD Auto-Tuning using the LAC's internal 10-bit ADC feedback
"""

from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout, QGroupBox,
    QComboBox, QLabel, QPushButton, QCheckBox, QMessageBox, QFrame, QSpinBox
)

from config.parameters import load_parameters, save_lac_config, DEFAULT_LAC_CONFIG
from connection.serial_manager import SerialManager
from ui.styles import (
    COLOR_BG_DARK, COLOR_PANEL_BG, COLOR_CARD_BG, COLOR_BORDER,
    COLOR_BORDER_LIGHT, COLOR_CYAN, COLOR_EMERALD, COLOR_EMERGENCY,
    COLOR_WARNING, COLOR_TEXT_PRIMARY, COLOR_TEXT_SECONDARY,
    FONT_MAIN, FONT_MONO
)


class SerialConfigDialog(QDialog):
    """
    Modal USB Hub LAC Configuration & Diagnostics Dialog.
    Replaces old Arduino Giga COM dialog with direct 6-LAC USB Hub management.
    """

    def __init__(self, serial_manager: SerialManager, parent=None):
        super().__init__(parent)
        self.serial_manager = serial_manager

        self.setWindowTitle("Configuración USB Actuonix LAC (Hub 6 Puertos)")
        self.setMinimumWidth(700)
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
                margin-top: 12px;
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
            QComboBox, QSpinBox {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 4px 8px;
                min-height: 20px;
                font-family: {FONT_MONO};
                font-size: 11px;
            }}
            QComboBox:hover, QSpinBox:hover {{
                border-color: {COLOR_CYAN};
            }}
            QPushButton {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 6px 14px;
                font-weight: 600;
                font-size: 11px;
            }}
            QPushButton:hover {{
                background-color: #222C38;
                border-color: {COLOR_CYAN};
            }}
            QPushButton#btn_connect {{
                background-color: #1B5E20;
                border: 1px solid #2E7D32;
                color: #FFFFFF;
                font-weight: 700;
            }}
            QPushButton#btn_connect:hover {{
                background-color: #2E7D32;
            }}
            QPushButton#btn_disconnect {{
                background-color: {COLOR_EMERGENCY};
                border: 1px solid #FF5252;
                color: #FFFFFF;
                font-weight: 700;
            }}
        """)

        self._setup_ui()
        self._load_current_lac_values()
        self._connect_signals()
        self.refresh_ports()
        self._update_status()
        self._update_devices_list(self.serial_manager.detected_devices)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 14, 16, 14)

        # 1. USB Hub Connection Group
        grp_conn = QGroupBox("1. Enlace USB Hub (6 Tarjetas Actuonix LAC)")
        form = QFormLayout(grp_conn)
        form.setSpacing(8)

        self.combo_ports = QComboBox()
        self.btn_refresh = QPushButton("↻ Escanear USB")
        self.btn_refresh.setToolTip("Buscar tarjetas Actuonix LAC conectadas al Hub USB (VID=0x04D8, PID=0xFC5F)")
        self.btn_refresh.clicked.connect(self._on_scan_usb_clicked)

        row_port = QHBoxLayout()
        row_port.addWidget(self.combo_ports, stretch=1)
        row_port.addWidget(self.btn_refresh)
        form.addRow("Modo / Dispositivo:", row_port)

        self.chk_simulation = QCheckBox("Modo Simulación Virtual (6-DOF Mock sin hardware físico)")
        self.chk_simulation.setStyleSheet(f"color: {COLOR_CYAN}; font-weight: 600; font-size: 11px;")
        self.chk_simulation.toggled.connect(self._on_simulation_toggled)
        form.addRow("", self.chk_simulation)

        # Status bar inside connection box
        self.status_card = QFrame()
        self.status_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 5px;
                padding: 6px;
            }}
        """)
        card_layout = QHBoxLayout(self.status_card)
        card_layout.setContentsMargins(8, 4, 8, 4)

        lbl_st_title = QLabel("ESTADO DEL ENLACE USB:")
        lbl_st_title.setStyleSheet(f"font-weight: 700; color: {COLOR_TEXT_SECONDARY}; font-size: 11px;")
        card_layout.addWidget(lbl_st_title)

        self.lbl_status = QLabel("DESCONECTADO")
        self.lbl_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 11px; color: {COLOR_EMERGENCY};")
        card_layout.addWidget(self.lbl_status)

        form.addRow(self.status_card)
        layout.addWidget(grp_conn)

        # 2. LAC Serial Number Identification Group (0x50 / 0x51)
        grp_id = QGroupBox("2. Identificación de Tarjetas en el Hub (0x50 SET_SERIAL_NUM / 0x51 GET_SERIAL_NUM)")
        id_layout = QVBoxLayout(grp_id)
        id_layout.setSpacing(6)

        self.lbl_detected_summary = QLabel("Tarjetas detectadas: 0/6")
        self.lbl_detected_summary.setStyleSheet(f"font-family: {FONT_MONO}; color: {COLOR_TEXT_SECONDARY}; font-size: 10px;")
        id_layout.addWidget(self.lbl_detected_summary)

        assign_row = QHBoxLayout()
        assign_row.addWidget(QLabel("Tarjeta USB:"))
        self.combo_raw_dev = QComboBox()
        self.combo_raw_dev.setMinimumWidth(220)
        assign_row.addWidget(self.combo_raw_dev, stretch=1)

        assign_row.addWidget(QLabel("Asignar a Actuador:"))
        self.combo_assign_axis = QComboBox()
        for ax in range(1, 7):
            self.combo_assign_axis.addItem(f"Actuador L{ax} (Serial ID = {ax})", ax)
        assign_row.addWidget(self.combo_assign_axis)

        self.btn_assign_id = QPushButton("Grabar ID (0x50)")
        self.btn_assign_id.setToolTip("Escribe el Serial Number (1..6) en la EEPROM de la tarjeta seleccionada")
        self.btn_assign_id.clicked.connect(self._on_assign_serial_clicked)
        assign_row.addWidget(self.btn_assign_id)

        id_layout.addLayout(assign_row)
        layout.addWidget(grp_id)

        # 3. Advanced LAC Configuration Parameters (Datasheet Page 5 & 6)
        grp_cfg = QGroupBox("3. Configuración Avanzada LAC (Se sube automáticamente al iniciar la app)")
        cfg_vbox = QVBoxLayout(grp_cfg)
        cfg_grid = QGridLayout()
        cfg_grid.setHorizontalSpacing(14)
        cfg_grid.setVerticalSpacing(6)

        self.spins_lac = {}
        fields = [
            ("accuracy", "Accuracy (0x01):", 0, 1023, 4, "Banda de precisión PD (4 ≈ ±0.39mm en 100mm)"),
            ("retract_limit", "Retract Limit (0x02):", 0, 1023, 0, "Límite inferior de retracción (0..1023)"),
            ("extend_limit", "Extend Limit (0x03):", 0, 1023, 1023, "Límite superior de extensión (0..1023)"),
            ("kp", "Proportional Gain Kp (0x0C):", 0, 1023, 1, "Ganancia proporcional del controlador PD interno"),
            ("kd", "Derivative Gain Kd (0x0D):", 0, 1023, 10, "Tasa diferencial ante condición de stall"),
            ("pwm_threshold", "PWM Threshold (0x06):", 0, 1023, 80, "Distancia al setpoint donde actúa el control PD"),
            ("max_pwm", "PWM Maximum / Speed (0x0A):", 0, 1023, 1023, "Velocidad máxima del actuador (1023 = 100%)"),
            ("min_pwm", "PWM Minimum (0x0B):", 0, 1023, 80, "PWM mínimo aplicado por el control PD"),
            ("movement_threshold", "Movement Thresh (0x04):", 0, 1023, 3, "Velocidad mínima antes de considerar stall"),
            ("stall_time", "Stall Time ms (0x05):", 0, 65535, 10000, "Tiempo en ms antes de apagar motor por stall"),
            ("derivative_threshold", "Derivative Thresh (0x07):", 0, 1023, 10, "Umbral de activación derivativa"),
            ("max_derivative", "Derivative Max (0x08):", 0, 1023, 1023, "Aporte máximo del término derivativo"),
            ("min_derivative", "Derivative Min (0x09):", 0, 1023, 0, "Aporte mínimo del término derivativo"),
            ("average_rc", "Average RC (0x0E):", 1, 64, 4, "Muestras de filtro RC"),
            ("average_adc", "Average ADC (0x0F):", 1, 64, 8, "Muestras de filtro del ADC de posición"),
        ]

        for idx, (key, label_txt, vmin, vmax, vdef, tip) in enumerate(fields):
            r = idx // 2
            c = (idx % 2) * 2
            lbl = QLabel(label_txt)
            lbl.setToolTip(tip)
            sp = QSpinBox()
            sp.setRange(vmin, vmax)
            sp.setValue(vdef)
            sp.setToolTip(tip)
            self.spins_lac[key] = sp
            cfg_grid.addWidget(lbl, r, c)
            cfg_grid.addWidget(sp, r, c + 1)

        cfg_vbox.addLayout(cfg_grid)

        # Axis inversion checkboxes
        inv_row = QHBoxLayout()
        inv_row.addWidget(QLabel("Invertir sentido (100% - pos):"))
        self.chks_invert = []
        for i in range(6):
            chk = QCheckBox(f"L{i+1}")
            self.chks_invert.append(chk)
            inv_row.addWidget(chk)
        inv_row.addStretch()
        cfg_vbox.addLayout(inv_row)

        # Config action buttons
        cfg_btn_row = QHBoxLayout()
        self.btn_autotune = QPushButton("🎯 Auto-Sintonizar PD (ADC)")
        self.btn_autotune.setToolTip(
            "Mide el ADC de 10 bits en escalones cerrados y sintoniza automáticamente "
            "Accuracy, PWM Threshold, Kp, Kd, Min PWM y Average ADC"
        )
        self.btn_autotune.setStyleSheet(f"color: {COLOR_CYAN}; font-weight: 700;")
        self.btn_autotune.clicked.connect(self._on_autotune_clicked)
        cfg_btn_row.addWidget(self.btn_autotune)

        self.btn_upload_ram = QPushButton("⬆ Guardar y Subir (RAM)")
        self.btn_upload_ram.setToolTip("Guarda estos parámetros y los envía a las 6 tarjetas LAC conectadas")
        self.btn_upload_ram.clicked.connect(lambda: self._apply_lac_config(save_eeprom=False))
        cfg_btn_row.addWidget(self.btn_upload_ram)

        self.btn_disable_manual = QPushButton("💾 Guardar EEPROM (0x30)")
        self.btn_disable_manual.setToolTip(
            "Envía 0x30 DISABLE_MANUAL: desactiva los 4 potenciómetros físicos y guarda en EEPROM de las LACs"
        )
        self.btn_disable_manual.clicked.connect(lambda: self._apply_lac_config(save_eeprom=True))
        cfg_btn_row.addWidget(self.btn_disable_manual)

        self.btn_reset_factory = QPushButton("↺ Reset Fábrica (0xFF)")
        self.btn_reset_factory.setToolTip("Envía 0xFF RESET: reactiva potenciómetros físicos y valores de fábrica")
        self.btn_reset_factory.clicked.connect(self._on_reset_factory_clicked)
        cfg_btn_row.addWidget(self.btn_reset_factory)

        cfg_vbox.addLayout(cfg_btn_row)
        layout.addWidget(grp_cfg)

        # 4. Bottom Dialog Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(10)

        self.btn_toggle_conn = QPushButton("CONECTAR")
        self.btn_toggle_conn.setObjectName("btn_connect")
        self.btn_toggle_conn.clicked.connect(self._toggle_connection)
        btn_layout.addWidget(self.btn_toggle_conn)

        self.btn_close = QPushButton("Cerrar")
        self.btn_close.clicked.connect(self.accept)
        btn_layout.addWidget(self.btn_close)

        layout.addLayout(btn_layout)

    def _load_current_lac_values(self, cfg_override: dict = None):
        if cfg_override is not None:
            cfg = dict(DEFAULT_LAC_CONFIG)
            cfg.update(cfg_override)
        else:
            params = load_parameters()
            cfg = dict(DEFAULT_LAC_CONFIG)
            if isinstance(params, dict) and "LAC_CONFIG" in params:
                cfg.update(params["LAC_CONFIG"])

        for key, sp in self.spins_lac.items():
            if key in cfg:
                sp.setValue(int(cfg[key]))

        inv = cfg.get("invert_axis", [False] * 6)
        for i in range(min(6, len(inv))):
            self.chks_invert[i].setChecked(bool(inv[i]))

    def _collect_lac_cfg(self) -> dict:
        cfg = {k: int(sp.value()) for k, sp in self.spins_lac.items()}
        cfg["invert_axis"] = [chk.isChecked() for chk in self.chks_invert]
        return cfg

    def _apply_lac_config(self, save_eeprom: bool = False):
        cfg = self._collect_lac_cfg()
        save_lac_config(cfg)
        self.serial_manager.upload_lac_configuration(cfg, save_eeprom=save_eeprom)
        mode_msg = (
            "guardada en parameters.json y enviada a EEPROM (0x30 DISABLE_MANUAL)"
            if save_eeprom
            else "guardada en parameters.json y aplicada en RAM"
        )
        QMessageBox.information(
            self,
            "Configuración LAC Aplicada",
            f"La configuración de las tarjetas Actuonix LAC fue {mode_msg}.",
        )

    def _on_autotune_clicked(self):
        if not self.serial_manager.is_connected:
            QMessageBox.warning(
                self,
                "Enlace desconectado",
                "Conecte primero el Hub USB con las tarjetas LAC antes de iniciar la auto-sintonización.",
            )
            return
        reply = QMessageBox.question(
            self,
            "Confirmar Auto-Sintonización PD (ADC 10-bit)",
            "Se ejecutará una rutina automática de escalones cortos (18% ↔ 38%)\n"
            "leyendo el ADC interno (0x10 GET_FEEDBACK) de las tarjetas LAC conectadas\n"
            "para sintonizar Accuracy, PWM Threshold, Kp, Kd, Min PWM y Average ADC.\n\n"
            "¿Desea iniciar la sintonización ahora?",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.btn_autotune.setEnabled(False)
            self.btn_autotune.setText("⏳ Sintonizando...")
            self.serial_manager.autotune_lac(save_eeprom=False)

    @Slot(dict)
    def _on_lac_config_updated(self, tuned_cfg: dict):
        self._load_current_lac_values(cfg_override=tuned_cfg)
        self.btn_autotune.setEnabled(True)
        self.btn_autotune.setText("🎯 Auto-Sintonizar PD (ADC)")
        if self.isVisible():
            QMessageBox.information(
                self,
                "Auto-Sintonización Completada",
                "Parámetros PD óptimos calculados desde el ADC de las LAC y guardados en parameters.json:\n\n"
                f"• Accuracy (0x01): {tuned_cfg.get('accuracy')} ADC\n"
                f"• PWM Threshold (0x06): {tuned_cfg.get('pwm_threshold')} ADC\n"
                f"• Kp (0x0C): {tuned_cfg.get('kp')} | Kd (0x0D): {tuned_cfg.get('kd')}\n"
                f"• PWM Minimum (0x0B): {tuned_cfg.get('min_pwm')}\n"
                f"• Average ADC (0x0F): {tuned_cfg.get('average_adc')}",
            )

    def _on_assign_serial_clicked(self):
        if self.combo_raw_dev.count() == 0:
            QMessageBox.warning(
                self,
                "Sin tarjetas detectadas",
                "Conecte primero el Hub USB con las tarjetas LAC y presione 'CONECTAR'.",
            )
            return
        raw_index = self.combo_raw_dev.currentData()
        new_serial = self.combo_assign_axis.currentData()
        if raw_index is None or new_serial is None:
            return
        self.serial_manager.assign_lac_serial(int(raw_index), int(new_serial))

    def _on_reset_factory_clicked(self):
        reply = QMessageBox.question(
            self,
            "Confirmar Reset de Fábrica (0xFF)",
            "¿Desea enviar 0xFF RESET a las tarjetas LAC conectadas?\n"
            "Esto reactivará los 4 potenciómetros físicos y restaurará los valores de fábrica.",
            QMessageBox.Yes | QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.serial_manager.reset_lac_factory()

    def _connect_signals(self):
        self.serial_manager.connected.connect(self._on_connected)
        self.serial_manager.disconnected.connect(self._on_disconnected)
        self.serial_manager.devices_updated.connect(self._update_devices_list)
        self.serial_manager.lac_config_updated.connect(self._on_lac_config_updated)

    @Slot(list)
    def _update_devices_list(self, devices: list):
        self.combo_raw_dev.clear()
        if not devices:
            n_bus = self.serial_manager.count_physical_lac_boards()
            if n_bus > 0 and not self.serial_manager.is_connected:
                self.lbl_detected_summary.setText(
                    f"Tarjetas LAC detectadas en bus USB: {n_bus}/6 (Presione 'CONECTAR' para abrirlas)"
                )
            else:
                self.lbl_detected_summary.setText("Tarjetas LAC activas en Hub USB: 0/6")
            return

        parts = []
        for d in devices:
            pct = (d.last_raw_pos / 1023.0) * 100.0
            label = f"USB #{d.raw_index} | Serial={d.serial_num} → L{d.assigned_axis} (ADC={d.last_raw_pos}/1023, {pct:.1f}%)"
            self.combo_raw_dev.addItem(label, d.raw_index)
            parts.append(f"L{d.assigned_axis}(ID={d.serial_num})")

        self.lbl_detected_summary.setText(
            f"Tarjetas LAC activas ({len(devices)}/6): " + ", ".join(parts)
        )

    def _on_scan_usb_clicked(self):
        self.refresh_ports()
        if not self.serial_manager.is_connected and not self.chk_simulation.isChecked():
            n_lac = self.serial_manager.count_physical_lac_boards()
            if n_lac > 0:
                save_lac_config(self._collect_lac_cfg())
                self.serial_manager.connect_port(self.combo_ports.currentText(), 115200)

    def refresh_ports(self):
        current = self.combo_ports.currentText()
        self.combo_ports.clear()
        ports = self.serial_manager.available_ports()
        self.combo_ports.addItems(ports)
        if current in ports:
            self.combo_ports.setCurrentText(current)
        elif self.serial_manager.current_port in ports:
            self.combo_ports.setCurrentText(self.serial_manager.current_port)
        self._update_devices_list(self.serial_manager.detected_devices)

    def _on_simulation_toggled(self, checked: bool):
        if checked:
            idx = self.combo_ports.findText("SIMULADOR VIRTUAL (6-DOF Mock)", Qt.MatchContains)
            if idx >= 0:
                self.combo_ports.setCurrentIndex(idx)
        else:
            if self.combo_ports.count() > 0 and "SIMULADOR" in self.combo_ports.currentText():
                for i in range(self.combo_ports.count()):
                    if "SIMULADOR" not in self.combo_ports.itemText(i):
                        self.combo_ports.setCurrentIndex(i)
                        break

    def _toggle_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect_port()
        else:
            # Always save current LAC spinbox settings before connecting so startup upload uses them
            save_lac_config(self._collect_lac_cfg())
            port = self.combo_ports.currentText()
            if not port:
                QMessageBox.warning(self, " Selección Vacía", "Seleccione el Hub USB LAC o el Modo Simulador.")
                return
            self.serial_manager.connect_port(port, 115200)

    @Slot(str, int)
    def _on_connected(self, port: str, baud: int):
        self._update_status()

    @Slot()
    def _on_disconnected(self):
        self._update_status()

    def _update_status(self):
        if self.serial_manager.is_connected:
            port_name = self.serial_manager.current_port or "HUB USB LAC"
            self.lbl_status.setText(f"CONECTADO: {port_name}")
            self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 11px; color: {COLOR_EMERALD};")
            self.btn_toggle_conn.setText("DESCONECTAR")
            self.btn_toggle_conn.setObjectName("btn_disconnect")
            self.btn_toggle_conn.setStyleSheet(f"background-color: {COLOR_EMERGENCY}; border: 1px solid #FF5252;")
            self.combo_ports.setEnabled(False)
            self.chk_simulation.setEnabled(False)
        else:
            self.lbl_status.setText("DESCONECTADO")
            self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 11px; color: {COLOR_EMERGENCY};")
            self.btn_toggle_conn.setText("CONECTAR")
            self.btn_toggle_conn.setObjectName("btn_connect")
            self.btn_toggle_conn.setStyleSheet("")
            self.combo_ports.setEnabled(True)
            self.chk_simulation.setEnabled(True)
