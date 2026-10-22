"""
Actuonix LAC (Linear Actuator Control Board) Direct USB Driver.
Implements the 3-byte packet protocol over USB Bulk Endpoints (0x01 OUT / 0x81 IN)
for controlling and reading feedback from 6 LAC boards connected via a USB Hub.

Datasheet reference:
  VID = 0x04D8 (Microchip Technology Inc.)
  PID = 0xFC5F (Actuonix / Firgelli LAC)
  Packet Format: [Control, Data_Low, Data_High] (3 bytes)
"""

from __future__ import annotations

import ctypes
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("lac_usb_driver")

# USB Identifiers for Actuonix LAC (Microchip PIC18F14K50 / PIC18F2550 Custom USB)
LAC_VID = 0x04D8
LAC_PID = 0xFC5F
EP_OUT = 0x01
EP_IN = 0x81

# Control Command Constants (Datasheet Page 6)
CMD_SET_ACCURACY = 0x01
CMD_SET_RETRACT_LIMIT = 0x02
CMD_SET_EXTEND_LIMIT = 0x03
CMD_SET_MOVEMENT_THRESHOLD = 0x04
CMD_SET_STALL_TIME = 0x05
CMD_SET_PWM_THRESHOLD = 0x06
CMD_SET_DERIVATIVE_THRESHOLD = 0x07
CMD_SET_DERIVATIVE_MAXIMUM = 0x08
CMD_SET_DERIVATIVE_MINIMUM = 0x09
CMD_SET_PWM_MAXIMUM = 0x0A
CMD_SET_PWM_MINIMUM = 0x0B
CMD_SET_PROPORTIONAL_GAIN = 0x0C
CMD_SET_DERIVATIVE_GAIN = 0x0D
CMD_SET_AVERAGE_RC = 0x0E
CMD_SET_AVERAGE_ADC = 0x0F

CMD_GET_FEEDBACK = 0x10
CMD_SET_POSITION = 0x20
CMD_SET_SPEED = 0x21
CMD_DISABLE_MANUAL = 0x30
CMD_SET_SERIAL_NUM = 0x50
CMD_GET_SERIAL_NUM = 0x51
CMD_RESET = 0xFF


def _get_usb_backend():
    """
    Returns the pyusb libusb1 backend for both Windows and Ubuntu/Linux.
    - On Linux (Ubuntu): prioritizes system libusb-1.0.so.0, then libusb_package.
    - On Windows: prioritizes bundled libusb_package (64-bit/32-bit DLL), then system libusb1.
    """
    import sys

    if sys.platform.startswith("linux"):
        try:
            import usb.backend.libusb1 as libusb1
            backend = libusb1.get_backend()
            if backend is not None:
                return backend
        except Exception:
            pass

    try:
        import libusb_package
        backend = libusb_package.get_libusb1_backend()
        if backend is not None:
            return backend
    except Exception:
        pass

    try:
        import usb.backend.libusb1 as libusb1
        return libusb1.get_backend()
    except Exception:
        return None


@dataclass
class LacDeviceInfo:
    """Metadata for a single detected Actuonix LAC board on the USB Hub."""
    raw_index: int
    serial_num: int          # Value read via 0x51 GET_SERIAL_NUM (0..255)
    assigned_axis: int       # 1..6 (Actuator L1..L6)
    bus: int = 0
    address: int = 0
    port_numbers: Tuple[int, ...] = ()
    backend_type: str = "pyusb"
    last_raw_pos: int = 0    # Last ADC feedback (0..1023)


class HubLock:
    """
    Re-entrant wrapper around threading.Lock supporting non-blocking try-acquire
    and .locked() state inspection.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._owner = None
        self._count = 0

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        me = threading.get_ident()
        if self._owner == me:
            self._count += 1
            return True
        if not blocking:
            if self._lock.acquire(blocking=False):
                self._owner = me
                self._count = 1
                return True
            return False
        if self._lock.acquire(blocking=True, timeout=timeout):
            self._owner = me
            self._count = 1
            return True
        return False

    def release(self):
        me = threading.get_ident()
        if self._owner != me:
            raise RuntimeError("Cannot release un-owned lock")
        self._count -= 1
        if self._count == 0:
            self._owner = None
            self._lock.release()

    def locked(self) -> bool:
        return self._lock.locked()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()


class LacUsbDevice:
    """Represents a single connected Actuonix LAC board over PyUSB or MPUSBAPI."""

    def __init__(self, dev_handle, raw_index: int, backend_type: str = "pyusb", dll_handles=None, hub=None):
        self.dev = dev_handle
        self.raw_index = raw_index
        self.backend_type = backend_type
        self.dll_handles = dll_handles  # (dll, out_handle, in_handle) if mpusbapi
        self.hub = hub
        self.serial_num: int = 0
        self.assigned_axis: int = raw_index + 1
        self.last_raw_pos: int = 0
        self.is_open: bool = True

    def transfer_packet(self, control: int, data: int = 0, timeout_ms: int = 80) -> Optional[Tuple[int, int]]:
        """
        Sends a 3-byte packet [control, data_low, data_high] and reads back the 3-byte response.
        Returns (resp_control, resp_value_16bit) or None on error.
        Enters hub.lock if hub is assigned; if the lock is held, does not start a second USB transfer.
        """
        if not self.is_open:
            return None

        if self.hub is not None and getattr(self.hub, "lock", None) is not None:
            if not self.hub.lock.acquire(blocking=False):
                return None
            try:
                return self._raw_transfer_packet(control, data, timeout_ms)
            finally:
                self.hub.lock.release()
        return self._raw_transfer_packet(control, data, timeout_ms)

    def _raw_transfer_packet(self, control: int, data: int = 0, timeout_ms: int = 80) -> Optional[Tuple[int, int]]:
        if not self.is_open:
            return None

        control_sent = control & 0xFF
        data_clamped = int(data) & 0xFFFF
        buf_out = bytes([
            control_sent,
            data_clamped & 0xFF,
            (data_clamped >> 8) & 0xFF,
        ])

        if self.backend_type == "pyusb":
            try:
                self.dev.write(EP_OUT, buf_out, timeout=timeout_ms)
                resp = self.dev.read(EP_IN, 3, timeout=timeout_ms)
                if resp is not None and len(resp) == 3:
                    r_ctrl = int(resp[0])
                    # Si el primer byte del eco es 0x10 y el comando enviado fue 0x20, o al revés,
                    # descarta el paquete y regresa None. No lo uses como posición.
                    if (r_ctrl == 0x10 and control_sent == 0x20) or (r_ctrl == 0x20 and control_sent == 0x10):
                        return None
                    r_val = int(resp[1]) | (int(resp[2]) << 8)
                    return r_ctrl, r_val
                else:
                    # No llegaron los 3 bytes: vacía el IN con una sola lectura corta y traga la excepción
                    try:
                        self.dev.read(EP_IN, 64, timeout=5)
                    except Exception:
                        pass
                    return None
            except Exception:
                # Si write o read vencen o lanzan excepción: vacía el IN de esa tarjeta con una sola lectura corta
                # (EP_IN, 64 bytes, timeout 5 ms) y traga la excepción. Luego regresa None. No uses clear_halt.
                try:
                    self.dev.read(EP_IN, 64, timeout=5)
                except Exception:
                    pass
                return None

        elif self.backend_type == "mpusbapi" and self.dll_handles:
            dll, out_h, in_h = self.dll_handles
            try:
                c_buf_out = (ctypes.c_ubyte * 3)(buf_out[0], buf_out[1], buf_out[2])
                c_buf_in = (ctypes.c_ubyte * 3)()
                actual_len = ctypes.c_ulong(0)
                w_ok = dll._MPUSBWrite(out_h, ctypes.byref(c_buf_out), 3, ctypes.byref(actual_len), timeout_ms)
                if not w_ok:
                    return None
                r_ok = dll._MPUSBRead(in_h, ctypes.byref(c_buf_in), 3, ctypes.byref(actual_len), timeout_ms)
                if r_ok and actual_len.value == 3:
                    r_ctrl = int(c_buf_in[0])
                    r_val = int(c_buf_in[1]) | (int(c_buf_in[2]) << 8)
                    return r_ctrl, r_val
            except Exception:
                return None

        return None

    def get_serial_num(self, timeout_ms: int = 150) -> Optional[int]:
        """0x51 GET_SERIAL_NUM: Reads the user-assigned serial number (0..255)."""
        res = self.transfer_packet(CMD_GET_SERIAL_NUM, 0, timeout_ms=timeout_ms)
        if res is not None:
            self.serial_num = res[1] & 0xFF
            return self.serial_num
        return None

    def set_serial_num(self, serial_num: int) -> bool:
        """0x50 SET_SERIAL_NUM: Writes a serial number (0..255) to LAC EEPROM."""
        sn = max(0, min(255, int(serial_num)))
        res = self.transfer_packet(CMD_SET_SERIAL_NUM, sn, timeout_ms=500)
        if res is not None:
            self.serial_num = sn
            return True
        return False

    def get_feedback_raw(self, timeout_ms: int = 80) -> Optional[int]:
        """0x10 GET_FEEDBACK: Reads current actuator ADC position (0..1023) with 80 ms timeout."""
        res = self.transfer_packet(CMD_GET_FEEDBACK, 0, timeout_ms=timeout_ms)
        if res is not None:
            val = max(0, min(1023, int(res[1])))
            self.last_raw_pos = val
            return val
        return None

    def set_position_raw(self, target_raw: int, timeout_ms: int = 80) -> Optional[int]:
        """
        0x20 SET_POSITION: Sends target position (0..1023) and receives current ADC feedback (0..1023)
        in the same 3-byte reply packet (per Datasheet Page 6 & 9) with 80 ms timeout.
        """
        raw = max(0, min(1023, int(round(target_raw))))
        res = self.transfer_packet(CMD_SET_POSITION, raw, timeout_ms=timeout_ms)
        if res is not None:
            fb = max(0, min(1023, int(res[1])))
            self.last_raw_pos = fb
            return fb
        return None

    def apply_configuration(self, cfg: dict, save_to_eeprom: bool = False) -> bool:
        """
        Uploads all configuration parameters (0x01..0x0F) from cfg dictionary to this LAC board.
        If save_to_eeprom is True, also sends 0x30 DISABLE_MANUAL to persist in EEPROM.
        """
        commands = [
            (CMD_SET_ACCURACY, int(cfg.get("accuracy", 4))),
            (CMD_SET_RETRACT_LIMIT, int(cfg.get("retract_limit", 0))),
            (CMD_SET_EXTEND_LIMIT, int(cfg.get("extend_limit", 1023))),
            (CMD_SET_MOVEMENT_THRESHOLD, int(cfg.get("movement_threshold", 3))),
            (CMD_SET_STALL_TIME, int(cfg.get("stall_time", 10000))),
            (CMD_SET_PWM_THRESHOLD, int(cfg.get("pwm_threshold", 80))),
            (CMD_SET_DERIVATIVE_THRESHOLD, int(cfg.get("derivative_threshold", 10))),
            (CMD_SET_DERIVATIVE_MAXIMUM, int(cfg.get("max_derivative", 1023))),
            (CMD_SET_DERIVATIVE_MINIMUM, int(cfg.get("min_derivative", 0))),
            (CMD_SET_PWM_MAXIMUM, int(cfg.get("max_pwm", 1023))),
            (CMD_SET_PWM_MINIMUM, int(cfg.get("min_pwm", 80))),
            (CMD_SET_PROPORTIONAL_GAIN, int(cfg.get("kp", 1))),
            (CMD_SET_DERIVATIVE_GAIN, int(cfg.get("kd", 10))),
            (CMD_SET_AVERAGE_RC, int(cfg.get("average_rc", 4))),
            (CMD_SET_AVERAGE_ADC, int(cfg.get("average_adc", 8))),
        ]
        ok_all = True
        for cmd, val in commands:
            if self.transfer_packet(cmd, val, timeout_ms=300) is None:
                ok_all = False

        if save_to_eeprom:
            if self.transfer_packet(CMD_DISABLE_MANUAL, 0, timeout_ms=500) is None:
                ok_all = False

        return ok_all

    def disable_manual_eeprom(self) -> bool:
        """0x30 DISABLE_MANUAL: Saves current configuration to EEPROM and disables manual pots."""
        return self.transfer_packet(CMD_DISABLE_MANUAL, 0, timeout_ms=500) is not None

    def reset_factory_defaults(self) -> bool:
        """0xFF RESET: Re-enables manual potentiometers and resets configuration to factory default."""
        return self.transfer_packet(CMD_RESET, 0, timeout_ms=500) is not None

    def close(self):
        if not self.is_open:
            return
        self.is_open = False
        if self.backend_type == "pyusb" and self.dev is not None:
            try:
                import usb.util
                usb.util.dispose_resources(self.dev)
            except Exception:
                pass
        elif self.backend_type == "mpusbapi" and self.dll_handles:
            dll, out_h, in_h = self.dll_handles
            try:
                dll._MPUSBClose(out_h)
                dll._MPUSBClose(in_h)
            except Exception:
                pass


class LacUsbHubController:
    """
    Manages up to 6 Actuonix LAC boards connected via a USB Hub.
    Handles discovery, serial-number to actuator-axis (L1..L6) mapping,
    automatic startup configuration upload, and 10-bit position/feedback I/O.
    """

    def __init__(self, log_callback: Optional[Callable[[str], None]] = None):
        self.lock = HubLock()
        self._usb_busy: bool = False
        self.log_callback = log_callback
        self.devices_by_axis: Dict[int, LacUsbDevice] = {}  # axis 1..6 -> LacUsbDevice
        self.raw_devices: List[LacUsbDevice] = []
        self.invert_axis: List[bool] = [False] * 6
        self._last_usb_io_time: float = 0.0
        self.failure_counts: Dict[int, int] = {i: 0 for i in range(1, 7)}
        self._axis_failed_logged: Dict[int, bool] = {i: False for i in range(1, 7)}

    @property
    def connected_count(self) -> int:
        return len(self.raw_devices)

    @property
    def is_busy(self) -> bool:
        return self._usb_busy or self.lock.locked()

    @property
    def last_usb_io_time(self) -> float:
        return self._last_usb_io_time

    def _on_axis_failure(self, axis_num: int):
        self.failure_counts[axis_num] = self.failure_counts.get(axis_num, 0) + 1
        if not self._axis_failed_logged.get(axis_num, False):
            self._axis_failed_logged[axis_num] = True
            msg = f"LAC eje {axis_num} sin respuesta"
            logger.warning(msg)
            if self.log_callback:
                try:
                    self.log_callback(msg)
                except Exception:
                    pass

    def _on_axis_success(self, axis_num: int):
        self._axis_failed_logged[axis_num] = False

    def scan_and_open(self) -> Tuple[int, List[str]]:
        """
        Scans USB bus for all connected Actuonix LAC boards (VID=0x04D8, PID=0xFC5F),
        opens handles, reads each board's serial number (0x51 GET_SERIAL_NUM),
        and maps them to axes 1..6.
        Returns (num_detected, log_messages).
        """
        if not self.lock.acquire(blocking=False):
            return 0, ["[USB-LAC] Bus USB ocupado, no se puede escanear."]

        self._usb_busy = True
        logs: List[str] = []
        try:
            self._close_all_internal()

            # 1. Try PyUSB (with bundled libusb-1.0 backend)
            try:
                import usb.core
                import usb.util

                backend = _get_usb_backend()
                found = list(usb.core.find(find_all=True, idVendor=LAC_VID, idProduct=LAC_PID, backend=backend))

                for idx, udev in enumerate(found):
                    try:
                        if os.name != "nt":
                            try:
                                if udev.is_kernel_driver_active(0):
                                    udev.detach_kernel_driver(0)
                            except Exception:
                                pass
                        try:
                            udev.set_configuration()
                        except Exception:
                            pass
                        usb.util.claim_interface(udev, 0)
                        lac_dev = LacUsbDevice(udev, raw_index=idx, backend_type="pyusb", hub=self)
                        sn = lac_dev.get_serial_num()
                        if sn is None:
                            sn = 0
                        lac_dev.get_feedback_raw()
                        self.raw_devices.append(lac_dev)
                    except Exception as e:
                        err_str = str(e)
                        if "Errno 13" in err_str or "Access denied" in err_str:
                            logs.append(
                                f"[USB-LAC] Permiso USB denegado en dispositivo #{idx} ({e}). "
                                "En Ubuntu ejecute: sudo bash setup_ubuntu.sh (o instale 99-actuonix-lac.rules)."
                            )
                        else:
                            logs.append(f"[USB-LAC] Advertencia al abrir dispositivo #{idx}: {e}")
            except Exception as e:
                logs.append(f"[USB-LAC] Aviso backend PyUSB: {e}")

            # 2. Fallback to mpusbapi.dll if PyUSB found 0 devices and mpusbapi.dll exists
            if len(self.raw_devices) == 0:
                self._try_scan_mpusbapi(logs)

            if not self.raw_devices:
                return 0, logs

            # 3. Map detected LAC boards to Axes 1..6 using their programmed Serial Number (0x51)
            self._map_devices_to_axes(logs)
            return len(self.raw_devices), logs
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

    def _try_scan_mpusbapi(self, logs: List[str]):
        """Optional fallback for 32-bit/64-bit mpusbapi.dll if present in workspace or system paths."""
        dll_candidates = [
            Path(__file__).resolve().parent / "mpusbapi.dll",
            Path(__file__).resolve().parent.parent / "mpusbapi.dll",
            Path("mpusbapi.dll"),
            Path(r"C:\Program Files (x86)\Actuonix\LAC Configuration Utility\mpusbapi.dll"),
            Path(r"C:\Windows\SysWOW64\mpusbapi.dll"),
            Path(r"C:\Windows\System32\mpusbapi.dll"),
        ]
        dll_path = next((p for p in dll_candidates if p.exists()), None)
        if dll_path is None:
            return

        try:
            dll = ctypes.CDLL(str(dll_path))
            vid_pid = b"vid_04d8&pid_fc5f"
            fn_count = getattr(dll, "_MPUSBGetDeviceCount", getattr(dll, "MPUSBGetDeviceCount", None))
            fn_open = getattr(dll, "_MPUSBOpen", getattr(dll, "MPUSBOpen", None))
            if fn_count is None or fn_open is None:
                return
            count = fn_count(vid_pid)
            for idx in range(min(count, 6)):
                out_h = fn_open(idx, vid_pid, b"\\MCHP_EP1", 0, 0)
                in_h = fn_open(idx, vid_pid, b"\\MCHP_EP1", 1, 0)
                if out_h != -1 and in_h != -1 and out_h != 0 and in_h != 0:
                    lac_dev = LacUsbDevice(None, raw_index=idx, backend_type="mpusbapi", dll_handles=(dll, out_h, in_h), hub=self)
                    lac_dev.get_serial_num()
                    lac_dev.get_feedback_raw()
                    self.raw_devices.append(lac_dev)
        except Exception as e:
            logs.append(f"[USB-LAC] No se pudo cargar mpusbapi.dll ({e}); usando driver PyUSB.")

    def _map_devices_to_axes(self, logs: List[str]):
        """
        Maps each detected LAC board to Actuator L1..L6:
        - Priority 1: Serial number in 1..6 maps directly to L1..L6.
        - Priority 2: Serial number in 0..5 (if distinct) maps to L1..L6 (serial + 1).
        - Priority 3: Any unmapped boards fill the remaining free slots (1..6) in USB order.
        """
        self.devices_by_axis.clear()
        unassigned: List[LacUsbDevice] = []

        serials = [d.serial_num for d in self.raw_devices]
        # Check if user numbered them 0..5 instead of 1..6
        use_zero_based = (
            len(set(serials)) == len(serials)
            and all(0 <= s <= 5 for s in serials)
            and 0 in serials
            and 6 not in serials
        )

        for dev in self.raw_devices:
            target_axis = (dev.serial_num + 1) if use_zero_based else dev.serial_num
            if 1 <= target_axis <= 6 and target_axis not in self.devices_by_axis:
                dev.assigned_axis = target_axis
                self.devices_by_axis[target_axis] = dev
                logs.append(
                    f"[USB-LAC] Tarjeta USB #{dev.raw_index} (Serial={dev.serial_num}) -> Asignada a Actuador L{target_axis}"
                )
            else:
                unassigned.append(dev)

        for dev in unassigned:
            for axis in range(1, 7):
                if axis not in self.devices_by_axis:
                    dev.assigned_axis = axis
                    self.devices_by_axis[axis] = dev
                    logs.append(
                        f"[USB-LAC] Tarjeta USB #{dev.raw_index} (Serial={dev.serial_num} sin ID único 1..6) "
                        f"-> Asignada temporalmente a L{axis}. (Use 'Asignar ID' en ⚙ CONFIG para fijar su número)."
                    )
                    break

    def upload_configuration_all(self, cfg: dict, save_to_eeprom: bool = False) -> Tuple[int, List[str]]:
        """Uploads configuration parameters to all connected LAC boards."""
        if not self.lock.acquire(blocking=False):
            return 0, ["[USB-LAC] Bus USB ocupado; transferencia en curso."]

        self._usb_busy = True
        ok_count = 0
        logs: List[str] = []
        try:
            inv = cfg.get("invert_axis", [False] * 6)
            if isinstance(inv, list) and len(inv) == 6:
                self.invert_axis = [bool(x) for x in inv]
            for axis in sorted(self.devices_by_axis.keys()):
                dev = self.devices_by_axis[axis]
                if dev.apply_configuration(cfg, save_to_eeprom=save_to_eeprom):
                    ok_count += 1
                else:
                    logs.append(f"[USB-LAC] Error subiendo configuración a L{axis} (USB #{dev.raw_index}).")

            mode_str = "RAM + EEPROM (0x30)" if save_to_eeprom else "RAM de trabajo"
            logs.append(
                f"[USB-LAC] Configuración subida a {ok_count}/{len(self.devices_by_axis)} tarjetas LAC ({mode_str})."
            )
            return ok_count, logs
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

    def percent_to_raw(self, axis_0based: int, pct: float) -> int:
        """Converts 0..100% stroke to 0..1023 LAC ADC/Setpoint value (handling axis inversion)."""
        p = float(np.clip(pct, 0.0, 100.0))
        if 0 <= axis_0based < 6 and self.invert_axis[axis_0based]:
            p = 100.0 - p
        return int(round((p / 100.0) * 1023.0))

    def raw_to_percent(self, axis_0based: int, raw_val: int) -> float:
        """Converts 0..1023 LAC ADC reading to 0.0..100.0% stroke (handling axis inversion)."""
        r = max(0, min(1023, int(raw_val)))
        pct = (float(r) / 1023.0) * 100.0
        if 0 <= axis_0based < 6 and self.invert_axis[axis_0based]:
            pct = 100.0 - pct
        return float(np.clip(pct, 0.0, 100.0))

    def set_all_positions_percent(self, pct_list: List[float], fallback_actual: np.ndarray) -> np.ndarray:
        """
        Sends 0x20 SET_POSITION to all 6 LAC boards sequentially with 80 ms timeout
        and collects their immediate ADC feedback returned by SET_POSITION (Datasheet Page 6).
        If transfer_packet returns None for an axis, that axis keeps its last good value,
        the failure is counted, and the loop continues to the next axis without aborting.
        Updates _last_usb_io_time upon completion.
        Returns a 6-element numpy array of actual percentages (0.0..100.0%).
        """
        if not self.lock.acquire(blocking=False):
            return np.array(fallback_actual, dtype=np.float64).copy()

        self._usb_busy = True
        actual_pct = np.array(fallback_actual, dtype=np.float64).copy()
        try:
            for axis_idx in range(6):
                axis_num = axis_idx + 1
                target_pct = float(pct_list[axis_idx])
                dev = self.devices_by_axis.get(axis_num)
                if dev is not None and dev.is_open:
                    raw_cmd = self.percent_to_raw(axis_idx, target_pct)
                    fb_raw = dev.set_position_raw(raw_cmd, timeout_ms=80)
                    if fb_raw is not None:
                        actual_pct[axis_idx] = self.raw_to_percent(axis_idx, fb_raw)
                        self._on_axis_success(axis_num)
                    else:
                        # Conserva el último valor bueno, cuenta el fallo y sigue con el siguiente eje
                        self._on_axis_failure(axis_num)
                else:
                    pass
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

        return actual_pct

    def set_single_position_percent(self, axis_1based: int, pct: float) -> Optional[float]:
        """Sends 0x20 SET_POSITION to a single LAC board (1..6) with 80 ms timeout."""
        if not self.lock.acquire(blocking=False):
            return None

        self._usb_busy = True
        axis_idx = axis_1based - 1
        res_pct = None
        try:
            dev = self.devices_by_axis.get(axis_1based)
            if dev is not None and dev.is_open:
                raw_cmd = self.percent_to_raw(axis_idx, pct)
                fb_raw = dev.set_position_raw(raw_cmd, timeout_ms=80)
                if fb_raw is not None:
                    res_pct = self.raw_to_percent(axis_idx, fb_raw)
                    self._on_axis_success(axis_1based)
                else:
                    self._on_axis_failure(axis_1based)
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

        return res_pct

    def read_all_feedback_percent(self, fallback_actual: np.ndarray) -> np.ndarray:
        """
        Polls 0x10 GET_FEEDBACK on all connected LAC boards sequentially with 80 ms timeout.
        If transfer_packet returns None for an axis, that axis keeps its last good value,
        the failure is counted, and the loop continues to the next axis without aborting.
        """
        if not self.lock.acquire(blocking=False):
            return np.array(fallback_actual, dtype=np.float64).copy()

        self._usb_busy = True
        actual_pct = np.array(fallback_actual, dtype=np.float64).copy()
        try:
            for axis_idx in range(6):
                axis_num = axis_idx + 1
                dev = self.devices_by_axis.get(axis_num)
                if dev is not None and dev.is_open:
                    fb_raw = dev.get_feedback_raw(timeout_ms=80)
                    if fb_raw is not None:
                        actual_pct[axis_idx] = self.raw_to_percent(axis_idx, fb_raw)
                        self._on_axis_success(axis_num)
                    else:
                        self._on_axis_failure(axis_num)
        finally:
            self._usb_busy = False
            self.lock.release()

        return actual_pct

    def get_devices_summary(self) -> List[LacDeviceInfo]:
        """Returns a list of LacDeviceInfo for UI display and diagnostics."""
        summary: List[LacDeviceInfo] = []
        for dev in self.raw_devices:
            bus = getattr(dev.dev, "bus", 0) or 0
            addr = getattr(dev.dev, "address", 0) or 0
            ports = getattr(dev.dev, "port_numbers", ()) or ()
            summary.append(
                LacDeviceInfo(
                    raw_index=dev.raw_index,
                    serial_num=dev.serial_num,
                    assigned_axis=dev.assigned_axis,
                    bus=int(bus),
                    address=int(addr),
                    port_numbers=tuple(ports) if ports else (),
                    backend_type=dev.backend_type,
                    last_raw_pos=dev.last_raw_pos,
                )
            )
        return summary

    def assign_device_serial(self, raw_index: int, new_axis_serial: int) -> Tuple[bool, str]:
        """
        Writes a new serial number (1..6) to the LAC board at raw_index using 0x50 SET_SERIAL_NUM,
        verifies it with 0x51 GET_SERIAL_NUM, and updates the axis mapping.
        """
        if not self.lock.acquire(blocking=False):
            return False, "[USB-LAC] Bus USB ocupado; intente nuevamente."

        self._usb_busy = True
        try:
            target_dev = next((d for d in self.raw_devices if d.raw_index == raw_index), None)
            if target_dev is None:
                return False, f"No se encontró la tarjeta USB #{raw_index}."

            if not (1 <= new_axis_serial <= 6):
                return False, "El ID de actuador debe estar entre 1 y 6."

            if not target_dev.set_serial_num(new_axis_serial):
                return False, f"Fallo al escribir comando 0x50 SET_SERIAL_NUM en USB #{raw_index}."

            readback = target_dev.get_serial_num()
            logs: List[str] = []
            self._map_devices_to_axes(logs)
            if readback == new_axis_serial:
                return True, f"Tarjeta USB #{raw_index} grabada exitosamente con Serial ID = {new_axis_serial} (Actuador L{new_axis_serial})."
            return True, f"Comando 0x50 enviado a USB #{raw_index} (Serial leído={readback})."
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

    def reset_factory_defaults_all(self) -> int:
        """Sends 0xFF RESET to all connected LAC boards."""
        if not self.lock.acquire(blocking=False):
            return 0
        self._usb_busy = True
        count = 0
        try:
            for dev in self.raw_devices:
                if dev.reset_factory_defaults():
                    count += 1
            return count
        finally:
            self._usb_busy = False
            self._last_usb_io_time = time.perf_counter()
            self.lock.release()

    def _close_all_internal(self):
        for dev in self.raw_devices:
            dev.close()
        self.raw_devices.clear()
        self.devices_by_axis.clear()

    def close_all(self):
        with self.lock:
            self._close_all_internal()
