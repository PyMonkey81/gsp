"""
Asynchronous Serial Communication Manager for 6-DOF Gough-Stewart Platform.
Built on PySide6 QThread / QSerialPort with robust mock/simulation fallback.
"""

import sys
import time
from typing import List, Optional
import numpy as np

from PySide6.QtCore import QObject, QThread, Signal, Slot, QTimer
try:
    from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo
    QT_SERIAL_AVAILABLE = True
except ImportError:
    QT_SERIAL_AVAILABLE = False


class SerialWorker(QObject):
    """
    Worker QObject running inside dedicated QThread for serial I/O and telemetry.
    """
    sig_connected = Signal(str, int)
    sig_disconnected = Signal()
    sig_telemetry = Signal(list)       # [q1, q2, q3, q4, q5, q6] feedback in mm
    sig_message = Signal(str)         # Informative log message
    sig_error = Signal(str)           # Error message
    sig_safe_mode = Signal(bool)      # Safe mode state

    def __init__(self):
        super().__init__()
        self._port: Optional[QSerialPort] = None
        self._is_simulated = False
        self._is_connected = False
        self._safe_mode = True

        # Target and actual actuator states for simulation / closed-loop
        self._q_target = np.array([332.0] * 6, dtype=np.float64)
        self._q_actual = np.array([332.0] * 6, dtype=np.float64)
        self._sim_timer: Optional[QTimer] = None

    @Slot()
    def init_worker(self):
        """Called once thread starts to initialize timers and buffers."""
        self._sim_timer = QTimer(self)
        self._sim_timer.setInterval(33)  # ~30 Hz telemetry loop
        self._sim_timer.timeout.connect(self._simulation_step)

    @Slot(str, int)
    def open_port(self, port_name: str, baudrate: int):
        """Open physical serial port or launch simulation mode."""
        if self._is_connected:
            self.close_port()

        if "SIMULADOR" in port_name.upper() or "MOCK" in port_name.upper() or not QT_SERIAL_AVAILABLE:
            self._is_simulated = True
            self._is_connected = True
            self._safe_mode = True
            if self._sim_timer:
                self._sim_timer.start()
            self.sig_connected.emit(port_name, baudrate)
            self.sig_message.emit(f"[SERIAL] Conectado en Modo Simulación: {port_name} @ {baudrate} baud.")
            self.sig_safe_mode.emit(True)
            return

        try:
            self._port = QSerialPort()
            self._port.setPortName(port_name)
            self._port.setBaudRate(baudrate)
            self._port.setDataBits(QSerialPort.DataBits.Data8)
            self._port.setParity(QSerialPort.Parity.NoParity)
            self._port.setStopBits(QSerialPort.StopBits.OneStop)
            self._port.setFlowControl(QSerialPort.FlowControl.NoFlowControl)

            if not self._port.open(QSerialPort.OpenModeFlag.ReadWrite):
                err_msg = self._port.errorString()
                self.sig_error.emit(f"[SERIAL] Fallo al abrir puerto {port_name}: {err_msg}")
                self._port = None
                return

            self._port.readyRead.connect(self._handle_ready_read)
            self._is_simulated = False
            self._is_connected = True
            self._safe_mode = True
            self.sig_connected.emit(port_name, baudrate)
            self.sig_message.emit(f"[SERIAL] Puerto físico conectado exitosamente: {port_name} @ {baudrate} baud.")
            self.sig_safe_mode.emit(True)
        except Exception as e:
            self.sig_error.emit(f"[SERIAL] Excepción al conectar: {str(e)}")

    @Slot()
    def close_port(self):
        """Close current connection."""
        if self._sim_timer and self._sim_timer.isActive():
            self._sim_timer.stop()

        if self._port and self._port.isOpen():
            try:
                self._port.close()
            except Exception:
                pass
            self._port = None

        self._is_connected = False
        self._is_simulated = False
        self.sig_disconnected.emit()
        self.sig_message.emit("[SERIAL] Puerto desconectado.")

    @Slot(list)
    def write_positions(self, q_mm: list):
        """Send actuator length commands to platform microcontroller."""
        if not self._is_connected:
            return

        q_arr = np.asarray(q_mm, dtype=np.float64)
        self._q_target = q_arr.copy()

        if self._is_simulated:
            # Target accepted in simulation
            return

        if self._port and self._port.isOpen():
            # Standard ASCII protocol: POS q1 q2 q3 q4 q5 q6\n (in mm with 2 decimals)
            msg = "POS " + " ".join(f"{val:.2f}" for val in q_arr) + "\n"
            try:
                self._port.write(msg.encode("ascii"))
            except Exception as e:
                self.sig_error.emit(f"[SERIAL] Error al enviar comando POS: {str(e)}")

    @Slot(str)
    def write_raw(self, cmd: str):
        """Send raw command string."""
        if not self._is_connected:
            return

        if self._is_simulated:
            self.sig_message.emit(f"[SIM-RX] Comando procesado: {cmd.strip()}")
            return

        if self._port and self._port.isOpen():
            try:
                line = cmd.strip() + "\n"
                self._port.write(line.encode("ascii"))
            except Exception as e:
                self.sig_error.emit(f"[SERIAL] Error al enviar comando: {str(e)}")

    @Slot()
    def emergency_stop(self):
        """Immediate emergency stop."""
        self._safe_mode = False
        if self._port and self._port.isOpen():
            try:
                self._port.write(b"ESTOP\n")
            except Exception:
                pass
        self.sig_safe_mode.emit(False)
        self.sig_message.emit("[EMERGENCY] !!! PARADA DE EMERGENCIA ACTIVADA !!!")

    @Slot()
    def reset_safe_mode(self):
        """Clear emergency stop and restore safe mode."""
        self._safe_mode = True
        if self._port and self._port.isOpen():
            try:
                self._port.write(b"RESET_SAFE\n")
            except Exception:
                pass
        self.sig_safe_mode.emit(True)
        self.sig_message.emit("[SAFE] Modo Seguro restaurado.")

    def _handle_ready_read(self):
        """Read incoming bytes from serial hardware and parse telemetry lines."""
        if not self._port:
            return

        while self._port.canReadLine():
            line_bytes = self._port.readLine()
            try:
                line = bytes(line_bytes).decode("ascii", errors="replace").strip()
                if not line:
                    continue
                # Telemetry line format: TEL q1 q2 q3 q4 q5 q6 (or CSV: q1,q2,q3,q4,q5,q6)
                if line.startswith("TEL") or line.startswith("FEEDBACK"):
                    parts = line.replace(",", " ").split()[1:]
                    if len(parts) >= 6:
                        vals = [float(p) for p in parts[:6]]
                        self.sig_telemetry.emit(vals)
                else:
                    self.sig_message.emit(f"[HW-RX] {line}")
            except Exception as e:
                self.sig_error.emit(f"[SERIAL] Error parseando datos: {e}")

    def _simulation_step(self):
        """Simulate realistic actuator dynamics with closed-loop response & noise."""
        if not self._is_simulated:
            return

        # Simulate first/second order filter towards target + tiny sensor noise
        alpha_filter = 0.22
        noise = np.random.normal(0.0, 0.08, size=6)
        self._q_actual += alpha_filter * (self._q_target - self._q_actual) + noise
        self.sig_telemetry.emit(self._q_actual.tolist())


class SerialManager(QObject):
    """
    Public thread-safe controller for serial communications.
    Runs worker on separate QThread to protect UI render loop.
    """
    # Forwarded signals for MainWindow
    connected = Signal(str, int)
    disconnected = Signal()
    telemetry_received = Signal(list)
    message_received = Signal(str)
    error_occurred = Signal(str)
    safe_mode_changed = Signal(bool)

    # Internal dispatch signals to worker
    _sig_open = Signal(str, int)
    _sig_close = Signal()
    _sig_write_pos = Signal(list)
    _sig_write_raw = Signal(str)
    _sig_estop = Signal()
    _sig_reset_safe = Signal()

    def __init__(self):
        super().__init__()
        self._thread = QThread()
        self._worker = SerialWorker()
        self._worker.moveToThread(self._thread)

        # Wire thread startup to worker initialization
        self._thread.started.connect(self._worker.init_worker)

        # Connect internal commands to worker slots
        self._sig_open.connect(self._worker.open_port)
        self._sig_close.connect(self._worker.close_port)
        self._sig_write_pos.connect(self._worker.write_positions)
        self._sig_write_raw.connect(self._worker.write_raw)
        self._sig_estop.connect(self._worker.emergency_stop)
        self._sig_reset_safe.connect(self._worker.reset_safe_mode)

        # Connect worker outputs to public signals
        self._worker.sig_connected.connect(self._on_connected)
        self._worker.sig_disconnected.connect(self._on_disconnected)
        self._worker.sig_telemetry.connect(self.telemetry_received)
        self._worker.sig_message.connect(self.message_received)
        self._worker.sig_error.connect(self.error_occurred)
        self._worker.sig_safe_mode.connect(self.safe_mode_changed)

        self._is_connected = False
        self._safe_mode = True
        self._current_port = ""

        self._thread.start()

    def _on_connected(self, port: str, baud: int):
        self._is_connected = True
        self._current_port = port
        self.connected.emit(port, baud)

    def _on_disconnected(self):
        self._is_connected = False
        self._current_port = ""
        self.disconnected.emit()

    def available_ports(self) -> List[str]:
        """
        List available physical COM/tty ports with a Virtual Simulator entry.
        """
        ports = []
        if QT_SERIAL_AVAILABLE:
            for p in QSerialPortInfo.availablePorts():
                name = p.portName()
                desc = p.description()
                ports.append(f"{name} ({desc})" if desc else name)

        # Always include the Virtual Simulator so UI is immediately functional
        ports.append("SIMULADOR VIRTUAL (6-DOF Mock)")
        return ports

    def connect_port(self, port_identifier: str, baudrate: int = 115200):
        """Initiate non-blocking connection."""
        clean_name = port_identifier.split()[0] if "(" in port_identifier else port_identifier
        self._sig_open.emit(clean_name, int(baudrate))

    def disconnect_port(self):
        """Disconnect current port."""
        self._sig_close.emit()

    def send_positions(self, q_mm):
        """
        Send actuator lengths (list or np.ndarray of shape (6,)).
        """
        if isinstance(q_mm, np.ndarray):
            q_list = q_mm.tolist()
        else:
            q_list = list(q_mm)
        self._sig_write_pos.emit(q_list)

    def send_command(self, cmd: str):
        """Send raw command string."""
        self._sig_write_raw.emit(cmd)

    def emergency_stop(self):
        """Trigger emergency stop immediately."""
        self._safe_mode = False
        self._sig_estop.emit()

    def reset_safe_mode(self):
        """Restore safe mode."""
        self._safe_mode = True
        self._sig_reset_safe.emit()

    def home(self):
        """Send home sequence."""
        self._sig_write_raw.emit("HOME")

    def calibrate(self):
        """Send calibration sequence."""
        self._sig_write_raw.emit("CALIBRATE")

    def zero_sensors(self):
        """Send zero sensors command."""
        self._sig_write_raw.emit("ZERO")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_safe_mode(self) -> bool:
        return self._safe_mode

    def close(self):
        """Clean shutdown of worker and thread."""
        self._sig_close.emit()
        self._thread.quit()
        self._thread.wait(1000)
