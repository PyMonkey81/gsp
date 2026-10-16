"""
SerialConfigDialog for Gough-Stewart 6-DOF Platform.
Modal dialog for configuring COM ports, baud rate, and virtual simulation mode.
"""

from typing import Optional
from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout, QGroupBox,
    QComboBox, QLabel, QPushButton, QCheckBox, QMessageBox, QFrame
)

from config.parameters import AVAILABLE_BAUDRATES, DEFAULT_BAUDRATE
from connection.serial_manager import SerialManager
from ui.styles import (
    COLOR_BG_DARK, COLOR_PANEL_BG, COLOR_CARD_BG, COLOR_BORDER,
    COLOR_BORDER_LIGHT, COLOR_CYAN, COLOR_EMERALD, COLOR_EMERGENCY,
    COLOR_WARNING, COLOR_TEXT_PRIMARY, COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED, FONT_MAIN, FONT_MONO
)


class SerialConfigDialog(QDialog):
    """
    Modal serial communication configuration dialog:
    - Port selection with refresh
    - Baud rate selection
    - Connect / Disconnect button
    - Simulation mode toggle
    - Live connection status badge
    """

    def __init__(self, serial_manager: SerialManager, parent=None):
        super().__init__(parent)
        self.serial_manager = serial_manager

        self.setWindowTitle("Configuración de Comunicación Serial")
        self.setMinimumWidth(460)
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
            QComboBox {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 6px 10px;
                min-height: 22px;
                font-family: {FONT_MONO};
                font-size: 11px;
            }}
            QComboBox:hover {{
                border-color: {COLOR_CYAN};
            }}
            QComboBox::drop-down {{
                border: none;
            }}
            QPushButton {{
                background-color: {COLOR_CARD_BG};
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: 4px;
                padding: 8px 18px;
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
            QPushButton#btn_disconnect:hover {{
                background-color: #FF1744;
            }}
        """)

        self._setup_ui()
        self._connect_signals()
        self.refresh_ports()
        self._update_status()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(18, 18, 18, 18)

        # Connection Setup Group
        grp_conn = QGroupBox("Parámetros de Enlace Serial")
        form = QFormLayout(grp_conn)
        form.setSpacing(10)

        # Port row
        self.combo_ports = QComboBox()
        self.btn_refresh = QPushButton("↻")
        self.btn_refresh.setFixedWidth(36)
        self.btn_refresh.setToolTip("Refrescar lista de puertos detectados")
        self.btn_refresh.clicked.connect(self.refresh_ports)

        row_port = QHBoxLayout()
        row_port.addWidget(self.combo_ports, stretch=1)
        row_port.addWidget(self.btn_refresh)
        form.addRow("Puerto COM / Dispositivo:", row_port)

        # Baudrate row
        self.combo_baud = QComboBox()
        for b in AVAILABLE_BAUDRATES:
            self.combo_baud.addItem(str(b))
        self.combo_baud.setCurrentText(str(DEFAULT_BAUDRATE))
        form.addRow("Velocidad (Baud Rate):", self.combo_baud)

        # Virtual simulation checkbox
        self.chk_simulation = QCheckBox("Modo Simulación Virtual (Mock sin hardware físico)")
        self.chk_simulation.setStyleSheet(f"color: {COLOR_CYAN}; font-weight: 600; font-size: 11px;")
        self.chk_simulation.toggled.connect(self._on_simulation_toggled)
        form.addRow("", self.chk_simulation)

        layout.addWidget(grp_conn)

        # Status Card
        self.status_card = QFrame()
        self.status_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_CARD_BG};
                border: 1px solid {COLOR_BORDER};
                border-radius: 6px;
                padding: 10px;
            }}
        """)
        card_layout = QHBoxLayout(self.status_card)
        card_layout.setContentsMargins(10, 8, 10, 8)

        lbl_st_title = QLabel("ESTADO DEL PUERTO:")
        lbl_st_title.setStyleSheet(f"font-weight: 700; color: {COLOR_TEXT_SECONDARY}; font-size: 11px;")
        card_layout.addWidget(lbl_st_title)

        self.lbl_status = QLabel("DESCONECTADO")
        self.lbl_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 12px; color: {COLOR_EMERGENCY};")
        card_layout.addWidget(self.lbl_status)

        layout.addWidget(self.status_card)

        # Action Buttons
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

    def _connect_signals(self):
        self.serial_manager.connected.connect(self._on_connected)
        self.serial_manager.disconnected.connect(self._on_disconnected)

    def refresh_ports(self):
        current = self.combo_ports.currentText()
        self.combo_ports.clear()
        ports = self.serial_manager.available_ports()
        self.combo_ports.addItems(ports)
        if current in ports:
            self.combo_ports.setCurrentText(current)
        elif self.serial_manager.current_port in ports:
            self.combo_ports.setCurrentText(self.serial_manager.current_port)

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
            port = self.combo_ports.currentText()
            baud = int(self.combo_baud.currentText())
            if not port:
                QMessageBox.warning(self, "Puerto Vacío", "Por favor seleccione un puerto válido o use el simulador virtual.")
                return
            self.serial_manager.connect_port(port, baud)

    @Slot(str, int)
    def _on_connected(self, port: str, baud: int):
        self._update_status()

    @Slot()
    def _on_disconnected(self):
        self._update_status()

    def _update_status(self):
        if self.serial_manager.is_connected:
            port_name = self.serial_manager.current_port or "CONECTADO"
            self.lbl_status.setText(f"CONECTADO: {port_name}")
            self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 11px; color: {COLOR_EMERALD};")
            self.btn_toggle_conn.setText("DESCONECTAR")
            self.btn_toggle_conn.setObjectName("btn_disconnect")
            self.btn_toggle_conn.setStyleSheet(f"background-color: {COLOR_EMERGENCY}; border: 1px solid #FF5252;")
            self.combo_ports.setEnabled(False)
            self.combo_baud.setEnabled(False)
            self.chk_simulation.setEnabled(False)
        else:
            self.lbl_status.setText("DESCONECTADO")
            self.lbl_status.setStyleSheet(f"font-weight: 800; font-size: 11px; color: {COLOR_EMERGENCY};")
            self.btn_toggle_conn.setText("CONECTAR")
            self.btn_toggle_conn.setObjectName("btn_connect")
            self.btn_toggle_conn.setStyleSheet("")
            self.combo_ports.setEnabled(True)
            self.combo_baud.setEnabled(True)
            self.chk_simulation.setEnabled(True)
