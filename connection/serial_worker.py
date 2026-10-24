"""
Asynchronous USB LAC Worker for PySide6.
Handles direct USB communication with 6 Actuonix LAC boards via a USB Hub,
automatic startup discovery & configuration upload, closed-loop ADC auto-tuning,
and virtual simulator fallback.
"""

import time
from typing import List, Optional
import numpy as np
from PySide6.QtCore import QObject, Signal, Slot, QTimer

from config.parameters import load_parameters, save_lac_config, DEFAULT_LAC_CONFIG
from connection.lac_usb_driver import LacUsbHubController
from connection.lac_autotuner import LacAutoTuner


class SerialWorker(QObject):
    connected = Signal(bool)
    message_received = Signal(str)
    telemetry_received = Signal(list)
    devices_updated = Signal(list)
    lac_config_updated = Signal(dict)
    incoming_processed = Signal()
    error = Signal(str)
    finished = Signal()

    def __init__(self):
        super().__init__()
        self.lac_hub = LacUsbHubController(log_callback=self._on_lac_hub_log)
        self.port_name = ""
        self.baudrate = 115200
        self.is_simulated = False
        self.is_usb_connected = False
        self._is_autotuning = False
        self._usb_busy = False
        self._pending_target_percent: Optional[List[float]] = None
        self._target_percent = np.zeros(6, dtype=np.float64)
        self._actual_percent = np.zeros(6, dtype=np.float64)
        self._last_usb_io_time = 0.0
        self._last_0x20_time = 0.0
        self._last_0x10_time = 0.0

        self._pending_timer = QTimer(self)
        self._pending_timer.setSingleShot(True)
        self._pending_timer.timeout.connect(self._process_pending_target)

    def _on_lac_hub_log(self, msg: str):
        self.message_received.emit(msg)

    def _load_lac_cfg(self) -> dict:
        params = load_parameters()
        cfg = dict(DEFAULT_LAC_CONFIG)
        if isinstance(params, dict) and "LAC_CONFIG" in params:
            cfg.update(params["LAC_CONFIG"])
        return cfg

    @Slot(str, int)
    def connect_port(self, port_name: str, baudrate: int = 115200):
        self.port_name = port_name
        self.baudrate = baudrate

        self.lac_hub.close_all()
        self.is_usb_connected = False
        self._pending_timer.stop()
        self._pending_target_percent = None
        self._usb_busy = False
        self._last_0x20_time = 0.0
        self._last_0x10_time = 0.0

        # 1. Handle Virtual Simulator Mode
        if "SIMULADOR" in port_name.upper() or "MOCK" in port_name.upper():
            self.is_simulated = True
            self.connected.emit(True)
            self.devices_updated.emit([])
            self.message_received.emit(f"[SIM] Conectado en Modo Simulador ({port_name})")
            return

        # 2. Direct USB Hub Discovery for 6 Actuonix LAC Boards
        self.is_simulated = False
        count, scan_logs = self.lac_hub.scan_and_open()
        for msg in scan_logs:
            self.message_received.emit(msg)

        if count == 0:
            self.connected.emit(False)
            self.devices_updated.emit([])
            self.error.emit(
                "[USB-LAC] No se detectaron tarjetas Actuonix LAC en el Hub USB (VID=0x04D8, PID=0xFC5F). "
                "Verifique la conexión USB o active el Modo Simulador."
            )
            return

        self.is_usb_connected = True
        self.message_received.emit(f"[USB-LAC] Se detectaron {count}/6 tarjetas LAC en el Hub USB.")

        # 3. Upload configuration automatically on startup/connect
        lac_cfg = self._load_lac_cfg()
        _, cfg_logs = self.lac_hub.upload_configuration_all(lac_cfg, save_to_eeprom=False)
        for msg in cfg_logs:
            self.message_received.emit(msg)

        # 4. Read initial real positions (0x10 GET_FEEDBACK)
        self._actual_percent = self.lac_hub.read_all_feedback_percent(self._actual_percent)
        self._target_percent = self._actual_percent.copy()
        now_init = time.perf_counter()
        self._last_0x10_time = now_init
        self._last_usb_io_time = now_init

        self.devices_updated.emit(self.lac_hub.get_devices_summary())
        self.connected.emit(True)
        self.telemetry_received.emit(self._actual_percent.tolist())

    @Slot()
    def disconnect_port(self):
        self._pending_timer.stop()
        self._pending_target_percent = None
        self._usb_busy = False
        self.lac_hub.close_all()
        self.is_usb_connected = False
        self.is_simulated = False
        self.devices_updated.emit([])
        self.connected.emit(False)
        self.message_received.emit("[USB-LAC] Hub USB desconectado.")

    @Slot(str)
    def send_command(self, command: str):
        cmd = command.strip()
        cmd_lower = cmd.lower()

        if cmd_lower == "calibrate":
            self.autotune_lac(save_eeprom=False)
            return

        if self.is_simulated:
            if cmd_lower == "home":
                self._target_percent[:] = 0.0
                self.message_received.emit("[SIM] Recibido: home (0% en 6 LACs)")
            elif cmd_lower == "adsinit":
                self.message_received.emit(f"[SIM] Configuración LAC aplicada ({cmd_lower})")
            elif cmd_lower == "estop":
                self._target_percent = self._actual_percent.copy()
                self.message_received.emit("[SIM] ESTOP: consigna igualada a posición actual")
            elif cmd_lower == "getpos":
                vals = [round(float(p), 2) for p in self._actual_percent]
                pos_str = "pos " + ",".join(str(v) for v in vals)
                self.message_received.emit(pos_str)
            elif cmd_lower.startswith("pos "):
                try:
                    parts = cmd[4:].split(",")
                    if len(parts) == 6:
                        self._target_percent = np.clip(
                            np.array([float(p) for p in parts], dtype=np.float64), 0.0, 100.0
                        )
                except Exception:
                    pass
            elif cmd_lower.startswith("a") and len(cmd_lower) > 1 and cmd_lower[1].isdigit():
                try:
                    parts = cmd_lower.split()
                    axis = int(parts[0][1:]) - 1
                    val = float(parts[1])
                    if 0 <= axis < 6:
                        self._target_percent[axis] = np.clip(val, 0.0, 100.0)
                        self.message_received.emit(f"[SIM] Actuador L{axis+1} -> {val:.1f}%")
                except Exception:
                    pass
            return

        if not self.is_usb_connected:
            self.error.emit("Hub USB LAC no conectado")
            return

        # Direct USB LAC command handling
        if cmd_lower == "ping":
            return

        if cmd_lower == "adsinit":
            self._usb_busy = True
            try:
                lac_cfg = self._load_lac_cfg()
                _, cfg_logs = self.lac_hub.upload_configuration_all(lac_cfg, save_to_eeprom=False)
                for msg in cfg_logs:
                    self.message_received.emit(msg)
                self.devices_updated.emit(self.lac_hub.get_devices_summary())
            finally:
                self._usb_busy = False
                self._process_pending_target()
            return

        if cmd_lower == "home":
            self.send_positions([0.0] * 6)
            self.message_received.emit("[USB-LAC] Comando HOME (0x20 SET_POSITION = 0) enviado a las 6 LACs.")
            return

        if cmd_lower == "estop":
            target = self._actual_percent.tolist()
            self._pending_target_percent = None
            if self._usb_busy or self.lac_hub.is_busy:
                self._pending_target_percent = target
                return
            self._execute_set_positions(target)
            self.message_received.emit("[USB-LAC] ESTOP ejecutado: 6 actuadores detenidos en su posición actual.")
            return

        if cmd_lower == "getpos":
            if not (self._usb_busy or self.lac_hub.is_busy):
                self._usb_busy = True
                try:
                    self._actual_percent = self.lac_hub.read_all_feedback_percent(self._actual_percent)
                    now_io = time.perf_counter()
                    self._last_0x10_time = now_io
                    self._last_usb_io_time = now_io
                finally:
                    self._usb_busy = False
                    self._process_pending_target()
            self.devices_updated.emit(self.lac_hub.get_devices_summary())
            vals = [round(float(p), 2) for p in self._actual_percent]
            pos_str = "pos " + ",".join(str(v) for v in vals)
            self.message_received.emit(pos_str)
            return

        if cmd_lower.startswith("pos "):
            try:
                parts = cmd[4:].split(",")
                if len(parts) == 6:
                    target_vals = [float(p) for p in parts]
                    self.send_positions(target_vals)
            except Exception as e:
                self.error.emit(f"[USB-LAC] Error enviando posiciones: {e}")
            return

        if cmd_lower.startswith("a") and len(cmd_lower) > 1 and cmd_lower[1].isdigit():
            try:
                parts = cmd_lower.split()
                axis_1based = int(parts[0][1:])
                val = float(np.clip(float(parts[1]), 0.0, 100.0))
                if 1 <= axis_1based <= 6:
                    self._target_percent[axis_1based - 1] = val
                    if self._usb_busy or self.lac_hub.is_busy:
                        self._pending_target_percent = self._target_percent.tolist()
                        return
                    self._usb_busy = True
                    try:
                        fb = self.lac_hub.set_single_position_percent(axis_1based, val)
                        now = time.perf_counter()
                        self._last_0x20_time = now
                        self._last_usb_io_time = now
                        if fb is not None:
                            self._actual_percent[axis_1based - 1] = fb
                        self.telemetry_received.emit(self._actual_percent.tolist())
                    finally:
                        self._usb_busy = False
                        self._process_pending_target()
                    raw_val = self.lac_hub.percent_to_raw(axis_1based - 1, val)
                    self.message_received.emit(
                        f"[USB-LAC] Actuador L{axis_1based} -> {val:.1f}% (0x20 SET_POSITION={raw_val}/1023)"
                    )
            except Exception as e:
                self.error.emit(f"[USB-LAC] Error en movimiento individual: {e}")
            return

    def _on_autotune_telemetry(self, pcts: list):
        self._actual_percent = np.asarray(pcts, dtype=np.float64)
        self._last_usb_io_time = time.perf_counter()
        self.telemetry_received.emit(self._actual_percent.tolist())

    @Slot(bool)
    def autotune_lac(self, save_eeprom: bool = False):
        """
        Runs the closed-loop PD & ADC filter auto-tuning routine using the LAC's internal
        10-bit ADC feedback (0x10 GET_FEEDBACK / 0x20 SET_POSITION).
        """
        if self._is_autotuning:
            self.message_received.emit("[AUTOTUNE] Ya hay una sintonización automática en curso.")
            return

        if self.is_simulated:
            self.message_received.emit("[AUTOTUNE-SIM] Simulando auto-sintonización PD de las 6 tarjetas LAC...")
            cfg = self._load_lac_cfg()
            cfg.update({
                "accuracy": 4,
                "pwm_threshold": 75,
                "kp": 2,
                "kd": 12,
                "min_pwm": 82,
                "average_adc": 8,
            })
            save_lac_config(cfg)
            self.lac_config_updated.emit(cfg)
            self.message_received.emit(
                "[AUTOTUNE-SIM] Sintonización simulada completada: "
                "accuracy=4, pwm_threshold=75, Kp=2, Kd=12, min_pwm=82, average_adc=8."
            )
            return

        if not self.is_usb_connected or self.lac_hub.connected_count == 0:
            self.error.emit(
                "[AUTOTUNE] No hay tarjetas Actuonix LAC conectadas por USB para sintonizar."
            )
            return

        self._is_autotuning = True
        self._usb_busy = True
        try:
            tuner = LacAutoTuner(
                hub=self.lac_hub,
                base_cfg=self._load_lac_cfg(),
                log_cb=self.message_received.emit,
                telemetry_cb=self._on_autotune_telemetry,
            )
            ok, tuned_cfg, _ = tuner.run_autotune(save_to_eeprom=save_eeprom)
            self._actual_percent = self.lac_hub.read_all_feedback_percent(self._actual_percent)
            self._target_percent = self._actual_percent.copy()
            self._last_usb_io_time = time.perf_counter()
            self.telemetry_received.emit(self._actual_percent.tolist())
            self.devices_updated.emit(self.lac_hub.get_devices_summary())
            if ok:
                self.lac_config_updated.emit(tuned_cfg)
        except Exception as e:
            self.error.emit(f"[AUTOTUNE] Error durante la sintonización automática: {e}")
        finally:
            self._usb_busy = False
            self._is_autotuning = False
            self._process_pending_target()

    @Slot(list)
    def send_positions(self, positions: list):
        if self._is_autotuning:
            return
        if len(positions) != 6:
            self.error.emit("Se requieren exactamente 6 posiciones")
            return

        target = np.clip(np.asarray(positions, dtype=np.float64), 0.0, 100.0)

        if self.is_simulated:
            self._target_percent = target
            return

        if not self.is_usb_connected:
            return

        now = time.perf_counter()
        # Si hay una transferencia en curso o si el último barrido 0x20 fue hace menos de 50 ms,
        # guarda solo el último setpoint en _pending_target_percent y regresa. No encoles una ráfaga.
        if self._usb_busy or self.lac_hub.is_busy or (now - self._last_0x20_time < 0.050):
            self._pending_target_percent = target.tolist()
            remaining_ms = int(max(1.0, (0.050 - (now - self._last_0x20_time)) * 1000.0) + 0.5)
            self._pending_timer.start(remaining_ms)
            return

        self._execute_set_positions(target.tolist())

    def _execute_set_positions(self, target_list: List[float]):
        if not self.is_usb_connected:
            return
        self._pending_timer.stop()
        self._target_percent = np.asarray(target_list, dtype=np.float64)
        self._usb_busy = True
        self._last_0x20_time = time.perf_counter()
        try:
            self._actual_percent = self.lac_hub.set_all_positions_percent(
                self._target_percent.tolist(), self._actual_percent
            )
            self._last_usb_io_time = time.perf_counter()
            self.telemetry_received.emit(self._actual_percent.tolist())
        finally:
            self._usb_busy = False
            self._process_pending_target()

    @Slot()
    def _process_pending_target(self):
        """
        _process_pending_target manda ese setpoint cuando ya pasaron 50 ms y no hay USB en curso.
        """
        if self._pending_target_percent is None or not self.is_usb_connected:
            return
        if self._usb_busy or self.lac_hub.is_busy:
            return
        now = time.perf_counter()
        elapsed = now - self._last_0x20_time
        if elapsed >= 0.050:
            self._pending_timer.stop()
            pending = self._pending_target_percent
            self._pending_target_percent = None
            self._execute_set_positions(pending)
        else:
            remaining_ms = int(max(1.0, (0.050 - elapsed) * 1000.0) + 0.5)
            self._pending_timer.start(remaining_ms)

    @Slot(dict, bool)
    def apply_lac_configuration(self, cfg: dict, save_eeprom: bool = False):
        if self.is_simulated:
            self.message_received.emit(
                f"[SIM] Configuración LAC guardada ({'RAM + EEPROM 0x30' if save_eeprom else 'RAM'})."
            )
            return

        if not self.is_usb_connected:
            self.message_received.emit(
                "[USB-LAC] Configuración guardada en archivo. Se subirá automáticamente al detectar las LACs."
            )
            return

        self._usb_busy = True
        try:
            _, logs = self.lac_hub.upload_configuration_all(cfg, save_to_eeprom=save_eeprom)
            for msg in logs:
                self.message_received.emit(msg)
            self.devices_updated.emit(self.lac_hub.get_devices_summary())
        finally:
            self._usb_busy = False
            self._process_pending_target()

    @Slot(int, int)
    def assign_lac_serial(self, raw_index: int, new_axis_serial: int):
        if not self.is_usb_connected:
            self.error.emit("[USB-LAC] Conecte primero las tarjetas LAC por USB para asignarles ID.")
            return
        self._usb_busy = True
        try:
            ok, msg = self.lac_hub.assign_device_serial(raw_index, new_axis_serial)
            if ok:
                self.message_received.emit(f"[USB-LAC] {msg}")
            else:
                self.error.emit(f"[USB-LAC] {msg}")
            self.devices_updated.emit(self.lac_hub.get_devices_summary())
        finally:
            self._usb_busy = False
            self._process_pending_target()

    @Slot()
    def reset_lac_factory(self):
        if not self.is_usb_connected:
            self.error.emit("[USB-LAC] No hay tarjetas LAC conectadas por USB.")
            return
        self._usb_busy = True
        try:
            count = self.lac_hub.reset_factory_defaults_all()
            self.message_received.emit(
                f"[USB-LAC] Comando 0xFF RESET (valores de fábrica y potenciómetros activos) enviado a {count} tarjetas."
            )
        finally:
            self._usb_busy = False
            self._process_pending_target()

    @Slot()
    def process_incoming(self):
        try:
            # 1. Regresa de inmediato si hay autotuning o si hay un 0x20 o 0x10 en curso (el timer no se acumula)
            if self._is_autotuning or self._usb_busy or self.lac_hub.is_busy:
                return
            if self.is_simulated:
                alpha_filt = 0.25
                noise = np.random.normal(0.0, 0.05, size=6)
                self._actual_percent = np.clip(
                    self._actual_percent + alpha_filt * (self._target_percent - self._actual_percent) + noise,
                    0.0,
                    100.0,
                )
                self.telemetry_received.emit(self._actual_percent.tolist())
                return

            if not self.is_usb_connected:
                return

            # Si hay un setpoint pendiente, intenta procesarlo
            if self._pending_target_percent is not None:
                self._process_pending_target()
                if self._usb_busy or self.lac_hub.is_busy:
                    return

            now = time.perf_counter()

            # 2. Si hubo un 0x20 en los últimos 200 ms, no mandes 0x10. La posición de la interfaz es el eco del 0x20.
            if now - self._last_0x20_time < 0.200:
                return

            # 3. Si no hubo 0x20 en los últimos 200 ms, un solo barrido 0x10 de las seis, sin pausa entre ellas, como máximo cada 50 ms.
            if now - self._last_0x10_time < 0.050:
                return

            # Lectura 0x10 protegida con _usb_busy
            self._usb_busy = True
            try:
                self._actual_percent = self.lac_hub.read_all_feedback_percent(self._actual_percent)
                now_after = time.perf_counter()
                self._last_0x10_time = now_after
                self._last_usb_io_time = now_after
                self.telemetry_received.emit(self._actual_percent.tolist())
            finally:
                self._usb_busy = False
                self._process_pending_target()
        finally:
            self.incoming_processed.emit()

    @Slot()
    def stop(self):
        self.disconnect_port()
        self.finished.emit()
