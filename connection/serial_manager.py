"""
SerialManager for Gough-Stewart Platform.
Thread-safe serial interface managing SerialWorker in a dedicated QThread.
"""

import sys
from pathlib import Path
from typing import List

from PySide6.QtCore import Q_ARG, QMetaObject, QObject, QThread, QTimer, Qt, Signal, Slot
from PySide6.QtSerialPort import QSerialPortInfo
from connection.serial_worker import SerialWorker

_LINUX_SKIP = ("ttyS", "ttyprintk")
_LINUX_PREF = ("ttyACM", "ttyUSB", "ttyAMA")


def _canonical_port_name(info: QSerialPortInfo) -> str:
    location = info.systemLocation() or ""
    name = info.portName() or ""
    if sys.platform.startswith("linux"):
        if location.startswith("/dev/"):
            return location
        if name and not name.startswith("/dev/"):
            return f"/dev/{name}"
    return name or location


class SerialManager(QObject):
    connected_changed = Signal(bool)
    connected = Signal(str, int)
    disconnected = Signal()
    message_received = Signal(str)
    error_occurred = Signal(str)
    telemetry_received = Signal(list)
    real_position_received = Signal(list)
    safe_mode_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.read_timer = QTimer(self)
        self.read_timer.setInterval(25)  # 40 Hz polling
        self.read_timer.timeout.connect(self.worker.process_incoming)

        # Timer de timeout para consulta de posición 'getpos'
        self._waiting_getpos = False
        self._getpos_timer = QTimer(self)
        self._getpos_timer.setInterval(1200)
        self._getpos_timer.setSingleShot(True)
        self._getpos_timer.timeout.connect(self._on_getpos_timeout)

        self.worker.connected.connect(self._on_worker_connected)
        self.worker.message_received.connect(self._on_worker_message)
        self.worker.telemetry_received.connect(self.telemetry_received)
        self.worker.error.connect(self.error_occurred)
        self.worker.finished.connect(self.thread.quit)

        self._is_connected = False
        self._current_port = None
        self._current_baud = 115200
        self._safe_mode = True

        self.thread.start()

    @Slot(str)
    def _on_worker_message(self, message: str):
        if not message:
            return
        msg = message.strip()
        if msg.lower() == "pong" or msg.startswith("TEL") or msg.startswith("FEEDBACK"):
            return

        # Parsear respuesta a getpos o telemetría pos: "pos v1,v2,v3,v4,v5,v6"
        if msg.startswith("pos "):
            payload = msg[4:].strip()
            parts = [p.strip() for p in payload.split(",") if p.strip()]
            if len(parts) == 6:
                try:
                    vals = [int(round(float(p))) for p in parts]
                    if self._waiting_getpos:
                        self._waiting_getpos = False
                        self._getpos_timer.stop()
                    self.real_position_received.emit(vals)
                    return
                except ValueError:
                    pass

        self.message_received.emit(message)

    def _on_worker_connected(self, state: bool):
        self._is_connected = state
        self.connected_changed.emit(state)
        if state:
            if not self.read_timer.isActive():
                self.read_timer.start()
            self.connected.emit(self._current_port or "PORT", self._current_baud)
            self.safe_mode_changed.emit(True)
        else:
            self.read_timer.stop()
            self._current_port = None
            self.disconnected.emit()
            self.safe_mode_changed.emit(True)

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def current_port(self) -> str | None:
        return self._current_port

    def available_ports(self) -> List[str]:
        """Returns clean port strings for UI QComboBox."""
        ports = []
        for info in QSerialPortInfo.availablePorts():
            name = _canonical_port_name(info)
            if not name:
                continue

            short = Path(name).name
            if sys.platform.startswith("linux") and short.startswith(_LINUX_SKIP):
                continue

            desc = info.description()
            port_label = f"{name} ({desc})" if desc else name
            ports.append(port_label)

        # Always offer virtual simulator for testing without hardware
        ports.append("SIMULADOR VIRTUAL (6-DOF Mock)")
        return ports

    def connect_port(self, port_name: str | None = None, baudrate: int = 115200):
        if not port_name:
            ports = self.available_ports()
            port_name = ports[0] if ports else "SIMULADOR VIRTUAL (6-DOF Mock)"

        self._current_port = port_name
        self._current_baud = int(baudrate)

        QMetaObject.invokeMethod(
            self.worker,
            "connect_port",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, port_name),
            Q_ARG(int, int(baudrate)),
        )

    def disconnect_port(self):
        QMetaObject.invokeMethod(self.worker, "disconnect_port", Qt.ConnectionType.QueuedConnection)

    def send_positions(self, positions: list):
        """
        Envía posiciones de actuadores (en porcentaje 0..100 o convertidas).
        Comando: pos p1,p2,p3,p4,p5,p6
        """
        if len(positions) != 6:
            self.error_occurred.emit("Se requieren exactamente 6 posiciones")
            return

        cmd = "pos " + ",".join(str(int(round(float(p)))) for p in positions)
        self.send_raw(cmd)

    def send_raw(self, command: str):
        """Envía comando ASCII crudo."""
        QMetaObject.invokeMethod(
            self.worker,
            "send_command",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, command),
        )

    def request_position(self):
        """
        Envía comando 'getpos' al firmware para consultar la posición actual de los 6 actuadores (0..100%).
        El firmware responde con 'pos v1,v2,v3,v4,v5,v6'.
        """
        if not self.is_connected:
            self.message_received.emit("sin lectura de posición")
            return
        if not self._waiting_getpos:
            self._waiting_getpos = True
            self._getpos_timer.start(1200)
        self.send_raw("getpos")

    def get_position(self):
        """Alias para request_position."""
        self.request_position()

    def _on_getpos_timeout(self):
        if self._waiting_getpos:
            self._waiting_getpos = False
            self.message_received.emit("sin lectura de posición")

    def send_single_actuator(self, axis_1based: int, value: int | float):
        """
        Envía comando individual de un solo actuador: 'aN valor'
        axis_1based: 1 a 6
        value: 0 a 100 (%)
        No envía 'pos' ni mueve los otros 5 ejes.
        """
        if not (1 <= axis_1based <= 6):
            self.error_occurred.emit(f"Eje fuera de rango (1..6): {axis_1based}")
            return
        val = int(round(float(value)))
        val = max(0, min(100, val))
        self.send_raw(f"a{axis_1based} {val}")

    def home(self):
        """Comando de retorno a home."""
        self.send_raw("home")

    def emergency_stop(self):
        """Parada de emergencia inmediata."""
        self._safe_mode = False
        self.safe_mode_changed.emit(False)
        self.send_raw("estop")

    def reset_safe_mode(self):
        self._safe_mode = True
        self.safe_mode_changed.emit(True)
        self.send_raw("reset_safe")

    def calibrate(self):
        self.send_raw("calibrate")

    def zero_sensors(self):
        self.send_raw("zero")

    def send_ping(self):
        """Envía ping para mantener viva la conexión en la Arduino."""
        if self.is_connected:
            self.send_raw("ping")

    def close(self):
        self.read_timer.stop()
        if self.thread.isRunning():
            QMetaObject.invokeMethod(self.worker, "stop", Qt.ConnectionType.QueuedConnection)
            self.thread.quit()
            self.thread.wait(1000)
