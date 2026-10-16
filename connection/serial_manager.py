"""
SerialManager for Gough-Stewart Platform.
Thread-safe serial interface managing SerialWorker in a dedicated QThread.
"""

import sys
from pathlib import Path
from typing import List

from PySide6.QtCore import Q_ARG, QMetaObject, QObject, QThread, QTimer, Qt, Signal
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
    safe_mode_changed = Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.read_timer = QTimer(self)
        self.read_timer.setInterval(25)  # 40 Hz polling
        self.read_timer.timeout.connect(self.worker.process_incoming)

        self.worker.connected.connect(self._on_worker_connected)
        self.worker.message_received.connect(self.message_received)
        self.worker.telemetry_received.connect(self.telemetry_received)
        self.worker.error.connect(self.error_occurred)
        self.worker.finished.connect(self.thread.quit)

        self._is_connected = False
        self._current_port = None
        self._current_baud = 115200
        self._safe_mode = True

        self.thread.start()

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

    def close(self):
        self.read_timer.stop()
        if self.thread.isRunning():
            QMetaObject.invokeMethod(self.worker, "stop", Qt.ConnectionType.QueuedConnection)
            self.thread.quit()
            self.thread.wait(1000)
