"""
SerialManager (USB LAC Hub Manager) for Gough-Stewart Platform.
Thread-safe interface managing direct USB communication with 6 Actuonix LAC boards
in a dedicated QThread, while preserving full API compatibility with the GUI.
"""

from typing import List

from PySide6.QtCore import Q_ARG, QMetaObject, QObject, QThread, QTimer, Qt, Signal, Slot
from connection.serial_worker import SerialWorker
from connection.lac_usb_driver import LAC_VID, LAC_PID, _get_usb_backend


class SerialManager(QObject):
    connected_changed = Signal(bool)
    connected = Signal(str, int)
    disconnected = Signal()
    message_received = Signal(str)
    error_occurred = Signal(str)
    telemetry_received = Signal(list)
    real_position_received = Signal(list)
    safe_mode_changed = Signal(bool)
    devices_updated = Signal(list)
    lac_config_updated = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)

        self.read_timer = QTimer(self)
        self.read_timer.setInterval(33)  # ~30 Hz feedback loop
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
        self.worker.devices_updated.connect(self._on_devices_updated)
        self.worker.lac_config_updated.connect(self.lac_config_updated)
        self.worker.error.connect(self.error_occurred)
        self.worker.finished.connect(self.thread.quit)

        self._is_connected = False
        self._current_port = None
        self._current_baud = 115200
        self._safe_mode = True
        self.detected_devices = []

        self.thread.start()

    @Slot(list)
    def _on_devices_updated(self, devices: list):
        self.detected_devices = list(devices)
        self.devices_updated.emit(self.detected_devices)

    @Slot(str)
    def _on_worker_message(self, message: str):
        if not message:
            return
        msg = message.strip()
        if msg.lower() == "pong" or msg.startswith("TEL") or msg.startswith("FEEDBACK"):
            return

        # Parsear respuesta a getpos: "pos v1,v2,v3,v4,v5,v6"
        if msg.startswith("pos "):
            payload = msg[4:].strip()
            parts = [p.strip() for p in payload.split(",") if p.strip()]
            if len(parts) == 6:
                try:
                    vals = [float(p) for p in parts]
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
            self.connected.emit(self._current_port or "HUB USB ACTUONIX LAC", self._current_baud)
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

    def count_physical_lac_boards(self) -> int:
        """Counts how many Actuonix LAC boards are currently visible on the USB bus (PyUSB or mpusbapi.dll)."""
        if self._is_connected and self.detected_devices:
            return len(self.detected_devices)
        try:
            import usb.core
            backend = _get_usb_backend()
            devs = list(usb.core.find(find_all=True, idVendor=LAC_VID, idProduct=LAC_PID, backend=backend))
            if len(devs) > 0:
                return len(devs)
        except Exception:
            pass
        # Fallback check via mpusbapi.dll (for 32-bit Python + native Actuonix driver)
        try:
            import ctypes
            from pathlib import Path
            for p in (
                Path(__file__).resolve().parent / "mpusbapi.dll",
                Path(__file__).resolve().parent.parent / "mpusbapi.dll",
                Path(r"C:\Program Files (x86)\Actuonix\LAC Configuration Utility\mpusbapi.dll"),
                Path(r"C:\Windows\SysWOW64\mpusbapi.dll"),
                Path(r"C:\Windows\System32\mpusbapi.dll"),
            ):
                if p.exists():
                    dll = ctypes.CDLL(str(p))
                    fn = getattr(dll, "_MPUSBGetDeviceCount", getattr(dll, "MPUSBGetDeviceCount", None))
                    if fn is not None:
                        c = int(fn(b"vid_04d8&pid_fc5f"))
                        if c > 0:
                            return c
        except Exception:
            pass
        return 0

    def available_ports(self) -> List[str]:
        """Returns USB Hub LAC option and Virtual Simulator for UI QComboBox."""
        n_lac = self.count_physical_lac_boards()
        ports = [
            f"HUB USB ACTUONIX LAC ({n_lac}/6 detectadas)",
            "SIMULADOR VIRTUAL (6-DOF Mock)",
        ]
        return ports

    def auto_connect_usb_on_startup(self):
        """
        Called automatically when the application starts:
        Checks if any Actuonix LAC boards are plugged into the USB Hub.
        If detected, connects to all of them and uploads the LAC configuration automatically.
        """
        n_lac = self.count_physical_lac_boards()
        if n_lac > 0:
            self.connect_port(f"HUB USB ACTUONIX LAC ({n_lac}/6 detectadas)")
        else:
            self.message_received.emit(
                "[USB-LAC] Inicio del sistema: 0 tarjetas LAC detectadas en el Hub USB "
                "(Conecte el Hub de 6 puertos y presione '⚙ CONFIG' -> 'Conectar', o use Modo Simulador)."
            )

    def connect_port(self, port_name: str | None = None, baudrate: int = 115200):
        if not port_name:
            ports = self.available_ports()
            port_name = ports[0]

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
        Envía consignas de posición a las 6 tarjetas LAC por USB (0.0..100.0% -> 0..1023 de 10 bits).
        """
        if len(positions) != 6:
            self.error_occurred.emit("Se requieren exactamente 6 posiciones")
            return

        cmd = "pos " + ",".join(f"{float(p):.3f}" for p in positions)
        self.send_raw(cmd)

    def send_raw(self, command: str):
        """Envía comando al Worker USB LAC."""
        QMetaObject.invokeMethod(
            self.worker,
            "send_command",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(str, command),
        )

    def upload_lac_configuration(self, cfg: dict, save_eeprom: bool = False):
        """Sube la configuración de control a las 6 tarjetas LAC conectadas."""
        QMetaObject.invokeMethod(
            self.worker,
            "apply_lac_configuration",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(dict, cfg),
            Q_ARG(bool, bool(save_eeprom)),
        )

    def autotune_lac(self, save_eeprom: bool = False):
        """
        Ejecuta la sintonización automática en lazo cerrado (PD + filtro ADC) de las tarjetas LAC
        usando el ADC interno de 10 bits (0x10 GET_FEEDBACK / 0x20 SET_POSITION).
        """
        QMetaObject.invokeMethod(
            self.worker,
            "autotune_lac",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(bool, bool(save_eeprom)),
        )

    def assign_lac_serial(self, raw_index: int, new_axis_serial: int):
        """Asigna un número de serie (1..6) con el comando 0x50 SET_SERIAL_NUM a una tarjeta LAC."""
        QMetaObject.invokeMethod(
            self.worker,
            "assign_lac_serial",
            Qt.ConnectionType.QueuedConnection,
            Q_ARG(int, int(raw_index)),
            Q_ARG(int, int(new_axis_serial)),
        )

    def reset_lac_factory(self):
        """Envía 0xFF RESET a las tarjetas LAC conectadas."""
        QMetaObject.invokeMethod(
            self.worker,
            "reset_lac_factory",
            Qt.ConnectionType.QueuedConnection,
        )

    def request_position(self):
        """
        Consulta la posición actual de los 6 actuadores mediante 0x10 GET_FEEDBACK.
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
        Envía consigna individual a un solo actuador (L1..L6) mediante 0x20 SET_POSITION.
        """
        if not (1 <= axis_1based <= 6):
            self.error_occurred.emit(f"Eje fuera de rango (1..6): {axis_1based}")
            return
        val = max(0.0, min(100.0, float(value)))
        self.send_raw(f"a{axis_1based} {val:.2f}")

    def home(self):
        """Comando de retorno a home (0% en los 6 actuadores)."""
        self.send_raw("home")

    def emergency_stop(self):
        """Parada de emergencia inmediata (congela los 6 actuadores en su posición actual)."""
        self._safe_mode = False
        self.safe_mode_changed.emit(False)
        self.send_raw("estop")

    def reset_safe_mode(self):
        self._safe_mode = True
        self.safe_mode_changed.emit(True)
        self.send_raw("reset_safe")

    def calibrate(self):
        """Ejecuta la auto-sintonización en lazo cerrado del control PD de las LAC."""
        self.autotune_lac(save_eeprom=False)

    def zero_sensors(self):
        self.send_raw("zero")

    def send_ping(self):
        if self.is_connected:
            self.send_raw("ping")

    def close(self):
        self.read_timer.stop()
        if self.thread.isRunning():
            QMetaObject.invokeMethod(self.worker, "stop", Qt.ConnectionType.QueuedConnection)
            self.thread.quit()
            self.thread.wait(1000)
