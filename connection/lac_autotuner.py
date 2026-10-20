"""
Auto-Tuning Engine for Actuonix LAC Boards over Direct USB (10-bit ADC Feedback).
Strictly follows the Actuonix LAC Datasheet (Pages 6-9) control laws:
  - 0x01 SET_ACCURACY: Deadband around setpoint (Value / 1024 * Stroke).
  - 0x04 SET_MOVEMENT_THRESHOLD & 0x07 SET_DERIVATIVE_THRESHOLD: Stall speed detection
    (Datasheet p.8: Derivative Threshold is normally set equal to Movement Threshold).
  - 0x06 SET_PWM_THRESHOLD: Distance around setpoint where PWM PD controller is active.
  - 0x0B SET_PWM_MINIMUM: Minimum PWM value applied by the PD control inside PWM_THRESHOLD.
  - 0x0C SET_PROPORTIONAL_GAIN (Kp): Proportional approach speed (reduce Kp to reduce overshoot).
  - 0x0D SET_DERIVATIVE_GAIN (Kd): Anti-stall incremental rate during stall/slow approach.
  - 0x0F SET_AVERAGE_ADC: ADC filter samples (Datasheet p.9: adds delay; keep low 4..8 for fast PD update).

All parameters are written ONLY to LAC RAM (0x01..0x0F) without calling 0x30 DISABLE_MANUAL
by default, avoiding EEPROM wear, and are persisted in config/parameters.json (LAC_CONFIG)
so the app uploads them to RAM automatically on every startup.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from config.parameters import DEFAULT_LAC_CONFIG, STROKE, save_lac_config
from connection.lac_usb_driver import LacUsbHubController


@dataclass
class StepMetrics:
    """Closed-loop step response metrics measured from the LAC internal 10-bit ADC."""
    overshoot_adc: float
    steady_error_adc: float
    settling_time_s: float
    max_speed_adc_s: float
    reversals: int
    cost: float


class LacAutoTuner:
    """
    Closed-loop Auto-Tuner for 1 to 6 Actuonix LAC boards connected via USB Hub.
    Operates synchronously across all connected boards (pure vertical Z translation when
    mounted on the 6-DOF Gough-Stewart platform) so it can be safely executed either with
    the full platform structure connected or with actuators on a test bench.
    """

    def __init__(
        self,
        hub: LacUsbHubController,
        base_cfg: Optional[dict] = None,
        log_cb: Optional[Callable[[str], None]] = None,
        telemetry_cb: Optional[Callable[[List[float]], None]] = None,
    ):
        self.hub = hub
        self.cfg = dict(DEFAULT_LAC_CONFIG)
        if base_cfg:
            self.cfg.update(base_cfg)
        self.log_cb = log_cb or (lambda msg: None)
        self.telemetry_cb = telemetry_cb

    def _log(self, msg: str):
        self.log_cb(msg)

    def _emit_telemetry_from_raw(self, raw_by_axis: Dict[int, int]):
        if self.telemetry_cb is None:
            return
        pcts = [0.0] * 6
        for ax in range(1, 7):
            raw_val = raw_by_axis.get(ax, 0)
            pcts[ax - 1] = self.hub.raw_to_percent(ax - 1, raw_val)
        self.telemetry_cb(pcts)

    def _read_all_raw(self) -> Dict[int, int]:
        """Reads 0x10 GET_FEEDBACK from all connected LAC boards (normalized for axis inversion)."""
        out_norm: Dict[int, int] = {}
        out_raw_for_ui: Dict[int, int] = {}
        for ax, dev in sorted(self.hub.devices_by_axis.items()):
            val = dev.get_feedback_raw()
            raw_fb = val if val is not None else dev.last_raw_pos
            out_raw_for_ui[ax] = raw_fb
            inv = self.hub.invert_axis[ax - 1] if 0 <= ax - 1 < len(self.hub.invert_axis) else False
            out_norm[ax] = (1023 - raw_fb) if inv else raw_fb
        self._emit_telemetry_from_raw(out_raw_for_ui)
        return out_norm

    def _command_all_raw(self, target_raw: int) -> Dict[int, int]:
        """
        Sends 0x20 SET_POSITION to all connected LAC boards simultaneously and reads back
        the immediate ADC position returned in the 3-byte reply packet (Datasheet Page 6).
        """
        out_norm: Dict[int, int] = {}
        out_raw_for_ui: Dict[int, int] = {}
        target_clamped = max(0, min(1023, int(round(target_raw))))
        for ax, dev in sorted(self.hub.devices_by_axis.items()):
            inv = self.hub.invert_axis[ax - 1] if 0 <= ax - 1 < len(self.hub.invert_axis) else False
            cmd_val = (1023 - target_clamped) if inv else target_clamped
            fb = dev.set_position_raw(cmd_val)
            raw_fb = fb if fb is not None else dev.last_raw_pos
            out_raw_for_ui[ax] = raw_fb
            out_norm[ax] = (1023 - raw_fb) if inv else raw_fb
        self._emit_telemetry_from_raw(out_raw_for_ui)
        return out_norm

    def _smooth_align_all(self, target_raw: int, step_adc: int = 35):
        """
        Gradually aligns all connected actuators to target_raw in small synchronous increments
        so the top platform never experiences mechanical twist if legs started at different lengths.
        """
        current_map = self._read_all_raw()
        if not current_map:
            return
        max_dist = max(abs(v - target_raw) for v in current_map.values())
        if max_dist > step_adc * 2:
            mean_start = int(round(np.mean(list(current_map.values()))))
            n_steps = max(2, int(max_dist // step_adc))
            for waypoint in np.linspace(mean_start, target_raw, n_steps):
                self._command_all_raw(int(round(waypoint)))
                time.sleep(0.18)
        self._wait_until_settled(target_raw, tolerance_adc=7, timeout_s=3.5)

    def _wait_until_settled(
        self,
        target_raw: int,
        tolerance_adc: int = 8,
        timeout_s: float = 3.5,
    ) -> Tuple[List[float], Dict[int, List[int]]]:
        """
        Commands target_raw and polls 0x10 GET_FEEDBACK at high rate until all boards
        stay within tolerance_adc and stop moving for 0.18s (or timeout_s expires).
        Returns (timestamps, history_by_axis).
        """
        axes = sorted(self.hub.devices_by_axis.keys())
        t0 = time.perf_counter()
        timestamps: List[float] = []
        history: Dict[int, List[int]] = {ax: [] for ax in axes}

        first_fb = self._command_all_raw(target_raw)
        timestamps.append(0.0)
        for ax in axes:
            history[ax].append(first_fb.get(ax, target_raw))

        settled_since: Optional[float] = None
        while True:
            now = time.perf_counter()
            elapsed = now - t0
            if elapsed >= timeout_s:
                break

            time.sleep(0.012)
            fb_map = self._read_all_raw()
            t_cur = time.perf_counter() - t0
            timestamps.append(t_cur)
            for ax in axes:
                history[ax].append(fb_map.get(ax, target_raw))

            all_settled = True
            for ax in axes:
                err = abs(history[ax][-1] - target_raw)
                recent_move = (
                    abs(history[ax][-1] - history[ax][-4])
                    if len(history[ax]) >= 4
                    else 999
                )
                if err > tolerance_adc and recent_move > 1:
                    all_settled = False
                    break

            if all_settled:
                if settled_since is None:
                    settled_since = t_cur
                elif (t_cur - settled_since) >= 0.18:
                    break
            else:
                settled_since = None

        return timestamps, history

    def _evaluate_step_response(
        self,
        start_raw: int,
        target_raw: int,
        accuracy_adc: int,
        timeout_s: float = 3.2,
    ) -> StepMetrics:
        """
        Executes a synchronous step from start_raw to target_raw and computes closed-loop metrics:
        overshoot, steady-state error, settling time, peak speed, reversals (hunting), and cost J.
        """
        timestamps, history = self._wait_until_settled(
            target_raw=target_raw,
            tolerance_adc=max(accuracy_adc + 2, 5),
            timeout_s=timeout_s,
        )
        t_arr = np.asarray(timestamps, dtype=np.float64)
        direction = 1.0 if target_raw >= start_raw else -1.0
        step_mag = max(1.0, abs(float(target_raw - start_raw)))

        max_overshoot = 0.0
        max_steady_err = 0.0
        max_settling_t = float(t_arr[-1]) if len(t_arr) > 0 else timeout_s
        peak_speed = 0.0
        total_reversals = 0

        for ax, samples in history.items():
            y = np.asarray(samples, dtype=np.float64)
            if len(y) < 3:
                continue

            signed_excursion = (y - float(target_raw)) * direction
            overshoot = float(max(0.0, np.max(signed_excursion)))
            max_overshoot = max(max_overshoot, overshoot)

            steady_err = float(abs(y[-1] - float(target_raw)))
            max_steady_err = max(max_steady_err, steady_err)

            if len(t_arr) >= 4:
                dt = np.maximum(1e-3, np.diff(t_arr))
                dy = np.diff(y)
                v = np.abs(dy / dt)
                if len(v) >= 3:
                    v_smooth = np.convolve(v, np.ones(3) / 3.0, mode="valid")
                    peak_speed = max(peak_speed, float(np.max(v_smooth)))
                else:
                    peak_speed = max(peak_speed, float(np.max(v)))

            band = max(float(accuracy_adc) + 1.0, 4.0)
            outside_idx = np.where(np.abs(y - float(target_raw)) > band)[0]
            if len(outside_idx) == 0:
                t_settle = 0.05
            else:
                last_out = int(outside_idx[-1])
                t_settle = float(t_arr[min(last_out + 1, len(t_arr) - 1)])
            max_settling_t = max(max_settling_t, t_settle)

            # Detect direction reversals (hunting) near the target
            near_mask = np.abs(y - float(target_raw)) < max(25.0, 0.25 * step_mag)
            y_near = y[near_mask]
            if len(y_near) >= 4:
                diffs = np.diff(y_near)
                sig_diffs = diffs[np.abs(diffs) >= 2.0]
                if len(sig_diffs) >= 2:
                    signs = np.sign(sig_diffs)
                    revs = int(np.sum(signs[1:] * signs[:-1] < 0))
                    total_reversals += revs

        excess_ss_err = max(0.0, max_steady_err - float(accuracy_adc))
        cost = (
            6.5 * max_overshoot
            + 25.0 * float(total_reversals)
            + 6.0 * excess_ss_err
            + 1.8 * max_steady_err
            + 10.0 * max_settling_t
        )

        return StepMetrics(
            overshoot_adc=max_overshoot,
            steady_error_adc=max_steady_err,
            settling_time_s=max_settling_t,
            max_speed_adc_s=peak_speed,
            reversals=total_reversals,
            cost=cost,
        )

    def run_autotune(self, save_to_eeprom: bool = False) -> Tuple[bool, dict, List[str]]:
        """
        Executes the 4-stage automatic tuning routine on all connected LAC boards in RAM,
        and saves the winning configuration to config/parameters.json.
        Returns (success, tuned_config_dict, summary_logs).
        """
        logs: List[str] = []

        def emit(msg: str):
            logs.append(msg)
            self._log(msg)

        if self.hub.connected_count == 0 or not self.hub.devices_by_axis:
            emit("[AUTOTUNE] Error: No hay tarjetas Actuonix LAC conectadas por USB.")
            return False, self.cfg, logs

        axes = sorted(self.hub.devices_by_axis.keys())
        axes_str = ", ".join(f"L{ax}" for ax in axes)
        mm_per_adc = STROKE / 1024.0  # Datasheet Page 7: Value / 1024 * Stroke

        emit("=" * 68)
        emit(f"[AUTOTUNE] Iniciando Sintonización Automática PD (en RAM) para: {axes_str}")
        emit("=" * 68)

        initial_raw_map = self._read_all_raw()
        init_avg_raw = int(round(np.mean(list(initial_raw_map.values())))) if initial_raw_map else 0

        work_cfg = dict(self.cfg)
        work_cfg["retract_limit"] = int(work_cfg.get("retract_limit", 0))
        work_cfg["extend_limit"] = int(work_cfg.get("extend_limit", 1023))
        work_cfg["max_pwm"] = 1023
        work_cfg["stall_time"] = int(work_cfg.get("stall_time", 10000))
        work_cfg["movement_threshold"] = 3
        work_cfg["derivative_threshold"] = 3  # Datasheet p.8: equal to movement_threshold
        work_cfg["average_rc"] = 4

        # ---------------------------------------------------------------------
        # STAGE 1: ADC Noise Floor & Filter Lag (0x10 GET_FEEDBACK & 0x0F AVERAGE_ADC)
        # Datasheet Page 9: Keep average_adc low (4..8) so filter delay does not cause PD overshoot.
        # ---------------------------------------------------------------------
        emit("[AUTOTUNE] Etapa 1/4: Midiendo ruido del ADC interno de 10 bits en reposo...")
        work_cfg["average_adc"] = 6
        self.hub.upload_configuration_all(work_cfg, save_to_eeprom=False)

        adc_samples: Dict[int, List[int]] = {ax: [] for ax in axes}
        for _ in range(30):
            snap = self._read_all_raw()
            for ax in axes:
                adc_samples[ax].append(snap.get(ax, 0))
            time.sleep(0.005)

        max_p2p = 0
        for ax in axes:
            arr = np.asarray(adc_samples[ax], dtype=np.float64)
            p2p = int(np.max(arr) - np.min(arr))
            std = float(np.std(arr))
            max_p2p = max(max_p2p, p2p)
            emit(
                f"  -> L{ax}: ADC inicial={int(arr[-1])}/1023 | Ruido σ={std:.2f} ADC, "
                f"Pico-a-Pico={p2p} ADC ({p2p * mm_per_adc:.2f} mm)"
            )

        # Select lowest average_adc that keeps noise controlled to minimize PD loop delay (Datasheet p.9)
        if max_p2p <= 1:
            best_avg_adc = 6
        elif max_p2p <= 2:
            best_avg_adc = 8
        else:
            best_avg_adc = 10

        base_accuracy = max(4, min(8, int(max_p2p) + 3))
        work_cfg["average_adc"] = best_avg_adc
        work_cfg["accuracy"] = base_accuracy
        emit(
            f"[AUTOTUNE] Filtro ADC óptimo: average_adc={best_avg_adc} (0x0F) | "
            f"Accuracy base={base_accuracy} ADC (±{base_accuracy * mm_per_adc:.2f} mm)"
        )

        # ---------------------------------------------------------------------
        # STAGE 2: Deadzone / Static Friction Identification (0x0B SET_PWM_MINIMUM)
        # ---------------------------------------------------------------------
        emit("[AUTOTUNE] Etapa 2/4: Alineando ejes y midiendo PWM mínimo de arranque (0x0B)...")
        start_raw = 184   # 18.0% stroke (level Z heave position)
        micro_raw = 216   # 21.1% stroke (+32 ADC micro-step)

        work_cfg["pwm_threshold"] = 85
        work_cfg["kp"] = 2
        work_cfg["kd"] = 10
        work_cfg["min_pwm"] = 80
        self.hub.upload_configuration_all(work_cfg, save_to_eeprom=False)

        self._smooth_align_all(start_raw)
        time.sleep(0.10)

        best_min_pwm = 80
        for candidate_pwm in (65, 80, 95, 110):
            for ax, dev in self.hub.devices_by_axis.items():
                dev.transfer_packet(0x0B, candidate_pwm)
            _, hist = self._wait_until_settled(micro_raw, tolerance_adc=base_accuracy + 1, timeout_s=0.65)
            errors = [abs(hist[ax][-1] - micro_raw) for ax in axes if len(hist[ax]) > 0]
            max_err = max(errors) if errors else 999
            if max_err <= base_accuracy + 2:
                best_min_pwm = min(130, candidate_pwm + 5)
                break
            best_min_pwm = candidate_pwm

        emit(f"[AUTOTUNE] Umbral mínimo PD determinado: min_pwm={best_min_pwm} (0x0B).")

        # ---------------------------------------------------------------------
        # STAGE 3: Step Response Speed & Braking Window (0x06 SET_PWM_THRESHOLD)
        # ---------------------------------------------------------------------
        emit("[AUTOTUNE] Etapa 3/4: Midiendo velocidad e inercia en escalón sincrónico (18% -> 35%)...")
        low_raw = 184    # 18.0%
        high_raw = 358   # 35.0% (Delta = 174 ADC ~= 17.0 mm pure vertical Z translation)

        work_cfg["min_pwm"] = best_min_pwm
        work_cfg["pwm_threshold"] = 80
        work_cfg["kp"] = 2
        work_cfg["kd"] = 10
        self.hub.upload_configuration_all(work_cfg, save_to_eeprom=False)

        m_initial = self._evaluate_step_response(
            start_raw=micro_raw,
            target_raw=high_raw,
            accuracy_adc=base_accuracy,
            timeout_s=3.5,
        )
        v_max = max(60.0, m_initial.max_speed_adc_s)
        v_mm_s = v_max * mm_per_adc
        emit(
            f"  -> Velocidad medida: {v_max:.1f} ADC/s ({v_mm_s:.1f} mm/s) | "
            f"Overshoot: {m_initial.overshoot_adc:.1f} ADC | "
            f"Error SS: {m_initial.steady_error_adc:.1f} ADC | t_s={m_initial.settling_time_s:.2f}s"
        )

        est_pwm_thresh = int(np.clip(round(0.28 * v_max), 45, 140))

        # ---------------------------------------------------------------------
        # STAGE 4: Closed-Loop Iterative PD Optimization (Datasheet Pages 7 & 8)
        # ---------------------------------------------------------------------
        emit("[AUTOTUNE] Etapa 4/4: Refinando lazo cerrado PD (pwm_threshold, Kp, Kd, accuracy)...")

        cand_cfg = dict(work_cfg)
        cand_cfg["pwm_threshold"] = est_pwm_thresh
        cand_cfg["min_pwm"] = best_min_pwm
        cand_cfg["kp"] = 1 if m_initial.overshoot_adc > 2.0 else 2
        cand_cfg["kd"] = 12 if m_initial.steady_error_adc > base_accuracy else 10
        cand_cfg["movement_threshold"] = 3
        cand_cfg["derivative_threshold"] = 3
        cand_cfg["accuracy"] = base_accuracy

        best_cfg = dict(cand_cfg)
        best_metrics = m_initial
        best_cost = float("inf")

        step_pairs = [
            (high_raw, low_raw),
            (low_raw, high_raw),
            (high_raw, low_raw),
        ]

        for it_idx, (s_raw, t_raw) in enumerate(step_pairs, start=1):
            self.hub.upload_configuration_all(cand_cfg, save_to_eeprom=False)
            metrics = self._evaluate_step_response(
                start_raw=s_raw,
                target_raw=t_raw,
                accuracy_adc=int(cand_cfg["accuracy"]),
                timeout_s=3.2,
            )

            emit(
                f"  [Iter {it_idx}/3] thr={cand_cfg['pwm_threshold']}, Kp={cand_cfg['kp']}, "
                f"Kd={cand_cfg['kd']}, min_pwm={cand_cfg['min_pwm']}, acc={cand_cfg['accuracy']} "
                f"-> OS={metrics.overshoot_adc:.1f} ADC, e_ss={metrics.steady_error_adc:.1f} ADC, "
                f"rev={metrics.reversals}, t_s={metrics.settling_time_s:.2f}s (J={metrics.cost:.1f})"
            )

            if metrics.cost < best_cost:
                best_cost = metrics.cost
                best_cfg = dict(cand_cfg)
                best_metrics = metrics

            # Datasheet Page 8 rules:
            # - Overshoot / reversals: reduce Kp, widen PWM_THRESHOLD braking zone, soften min_pwm
            # - Steady-state error > accuracy: increase Kp, min_pwm, and Kd anti-stall rate
            if metrics.overshoot_adc > 2.0 or metrics.reversals > 0:
                cand_cfg["pwm_threshold"] = min(170, int(cand_cfg["pwm_threshold"]) + 20)
                if int(cand_cfg["kp"]) > 1:
                    cand_cfg["kp"] = int(cand_cfg["kp"]) - 1
                cand_cfg["min_pwm"] = max(55, int(cand_cfg["min_pwm"]) - 8)
                if metrics.reversals > 0:
                    cand_cfg["accuracy"] = min(8, int(cand_cfg["accuracy"]) + 1)
            elif metrics.steady_error_adc > float(cand_cfg["accuracy"]):
                cand_cfg["kp"] = min(5, int(cand_cfg["kp"]) + 1)
                cand_cfg["min_pwm"] = min(140, int(cand_cfg["min_pwm"]) + 10)
                cand_cfg["kd"] = min(30, int(cand_cfg["kd"]) + 4)
                cand_cfg["pwm_threshold"] = max(45, int(cand_cfg["pwm_threshold"]) - 10)
            else:
                cand_cfg["pwm_threshold"] = max(45, int(cand_cfg["pwm_threshold"]) - 8)
                cand_cfg["kp"] = min(4, int(cand_cfg["kp"]) + 1)

        # ---------------------------------------------------------------------
        # STAGE 5: Apply Optimal Config to RAM, Return Smoothly & Save to JSON
        # ---------------------------------------------------------------------
        self.hub.upload_configuration_all(best_cfg, save_to_eeprom=save_to_eeprom)

        restore_target = max(0, min(1023, init_avg_raw))
        emit(f"[AUTOTUNE] Regresando actuadores a su posición inicial ({restore_target}/1023 ADC)...")
        self._smooth_align_all(restore_target)

        # Save to config/parameters.json so startup always loads these tuned values into RAM
        save_lac_config(best_cfg)

        acc_mm = int(best_cfg["accuracy"]) * mm_per_adc
        err_mm = best_metrics.steady_error_adc * mm_per_adc
        emit("-" * 68)
        emit("[AUTOTUNE] SINTONIZACIÓN GUARDADA EN config/parameters.json Y CARGADA EN RAM:")
        emit(
            f"  • Accuracy (0x01)         = {best_cfg['accuracy']} ADC (±{acc_mm:.2f} mm)\n"
            f"  • PWM Threshold (0x06)    = {best_cfg['pwm_threshold']} ADC (zona PD activa)\n"
            f"  • Proportional Gain (0x0C)= {best_cfg['kp']} (Kp)\n"
            f"  • Derivative Gain (0x0D)  = {best_cfg['kd']} (Kd anti-stall)\n"
            f"  • PWM Minimum (0x0B)      = {best_cfg['min_pwm']} / 1023\n"
            f"  • Movement/Deriv Thresh   = {best_cfg['movement_threshold']} (0x04 / 0x07)\n"
            f"  • Average ADC (0x0F)      = {best_cfg['average_adc']} muestras\n"
            f"  • Desempeño medido        = Error SS: {best_metrics.steady_error_adc:.1f} ADC ({err_mm:.2f} mm), "
            f"Overshoot: {best_metrics.overshoot_adc:.1f} ADC, t_s: {best_metrics.settling_time_s:.2f} s"
        )
        emit("=" * 68)

        return True, best_cfg, logs
