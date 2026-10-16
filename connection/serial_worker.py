"""
Asynchronous Serial Worker for PySide6.
Handles physical serial I/O via QSerialPort with virtual simulator fallback.
"""

import os
import time
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot, QTimer
from PySide6.QtSerialPort import QSerialPort


class SerialWorker(QObject):
    connected = Signal(bool)
    message_received = Signal(str)
    telemetry_received = Signal(list)
    error = Signal(str)
    finished = Signal()

    def __init__(self):
        super().__init__()
        self.serial = None
        self.port_name = ""
        self.baudrate = 115200
        self.is_simulated = False
        self._target_percent = np.zeros(6, dtype=np.float64)
        self._actual_percent = np.zeros(6, dtype=np.float64)

    @Slot(str, int)
    def connect_port(self, port_name: str, baudrate: int = 115200):
        self.port_name = port_name
        self.baudrate = baudrate

        if self.serial and self.serial.isOpen():
            self.serial.close()

        # Handle Mock / Simulator
        if "SIMULADOR" in port_name.upper() or "MOCK" in port_name.upper():
            self.is_simulated = True
            self.connected.emit(True)
            self.message_received.emit(f"[SIM] Conectado en Modo Simulador ({port_name})")
            return

        self.is_simulated = False
        name = port_name.split()[0]
        if os.name != "nt" and name and not name.startswith("/dev/") and not name.startswith("\\\\"):
            name = f"/dev/{name}"

        self.serial = QSerialPort()
        self.serial.setPortName(name)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(QSerialPort.DataBits.Data8)
        self.serial.setParity(QSerialPort.Parity.NoParity)
        self.serial.setStopBits(QSerialPort.StopBits.OneStop)
        self.serial.setFlowControl(QSerialPort.FlowControl.NoFlowControl)

        if self.serial.open(QSerialPort.OpenModeFlag.ReadWrite):
            time.sleep(1.2)
            self.serial.clear()
            self.connected.emit(True)
            self.serial.setDataTerminalReady(True)
            self.message_received.emit(f"[SERIAL] Puerto {name} abierto a {baudrate} baud.")
            return

        self.connected.emit(False)
        err = self.serial.error()
        detail = self.serial.errorString()
        self.error.emit(f"No se pudo abrir {name} (error={err}): {detail}")

    @Slot()
    def disconnect_port(self):
        if self.serial and self.serial.isOpen():
            self.serial.close()
        self.serial = None
        self.is_simulated = False
        self.connected.emit(False)
        self.message_received.emit("[SERIAL] Puerto desconectado.")

    @Slot(str)
    def send_command(self, command: str):
        if self.is_simulated:
            cmd = command.strip()
            if cmd == "home":
                self._target_percent[:] = 0.0
                self.message_received.emit("[SIM] Recibido: home")
            elif cmd.startswith("pos "):
                try:
                    parts = cmd[4:].split(",")
                    if len(parts) == 6:
                        self._target_percent = np.array([float(p) for p in parts], dtype=float)
                except Exception:
                    pass
            return

        if self.serial and self.serial.isOpen():
            if not command.endswith("\n"):
                command += "\n"
            self.serial.write(command.encode("utf-8"))
            self.serial.flush()
            return

        self.error.emit("Puerto no conectado")

    @Slot(list)
    def send_positions(self, positions: list):
        if len(positions) != 6:
            self.error.emit("Se requieren exactamente 6 posiciones")
            return

        if self.is_simulated:
            self._target_percent = np.asarray(positions, dtype=float)
            return

        # Authentic protocol: pos p1,p2,p3,p4,p5,p6\n
        cmd = "pos " + ",".join(str(int(round(p))) for p in positions)
        self.send_command(cmd)

    def process_incoming(self):
        if self.is_simulated:
            # Smooth simulated closed-loop actuator dynamics
            alpha_filt = 0.25
            noise = np.random.normal(0.0, 0.05, size=6)
            self._actual_percent += alpha_filt * (self._target_percent - self._actual_percent) + noise
            self.telemetry_received.emit(self._actual_percent.tolist())
            return

        if not self.serial or not self.serial.isOpen():
            return

        while self.serial.canReadLine():
            data = self.serial.readLine().data().decode("utf-8", errors="ignore").strip()
            if data:
                self.message_received.emit(data)
                # Parse telemetry feedback if available
                if data.startswith("TEL") or data.startswith("FEEDBACK"):
                    parts = data.replace(",", " ").split()[1:]
                    if len(parts) >= 6:
                        vals = [float(p) for p in parts[:6]]
                        self.telemetry_received.emit(vals)

    @Slot()
    def stop(self):
        self.disconnect_port()
        self.finished.emit()
