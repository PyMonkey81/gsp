"""
Sintonía de una sola LAC. Pensado para el eje 3, ya desacoplado de la plataforma.

Cierra la interfaz antes de correrlo. Si la app sigue abierta, el USB está ocupado.

    python tune_eje3.py
    python tune_eje3.py --eje 3

No escribe EEPROM y no toca parameters.json, salvo que al final contestes que sí.
La configuración es la misma para las seis tarjetas: si la guardas, la próxima
vez que abras la app se la sube también a L1, L2, L4, L5 y L6.
"""

import argparse
import time

from connection.lac_usb_driver import (
    EP_IN,
    EP_OUT,
    LacUsbHubController,
)
from config.parameters import DEFAULT_LAC_CONFIG, load_parameters


def raw_xfer(dev, control, data=0, timeout_ms=400):
    """
    Manda 3 bytes y enseña la respuesta tal cual.
    No tira el paquete si el primer byte no coincide: eso es lo que la app
    está contando como 'sin respuesta'.
    """
    buf = bytes([
        control & 0xFF,
        int(data) & 0xFF,
        (int(data) >> 8) & 0xFF,
    ])
    t0 = time.perf_counter()
    try:
        dev.dev.write(EP_OUT, buf, timeout=timeout_ms)
        resp = bytes(dev.dev.read(EP_IN, 3, timeout=timeout_ms))
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if len(resp) != 3:
            return None
        ctrl = resp[0]
        val = resp[1] | (resp[2] << 8)
        return {
            "ctrl": ctrl,
            "val": val,
            "raw": resp,
            "ms": dt_ms,
            "error": None,
        }
    except Exception as exc:
        dt_ms = (time.perf_counter() - t0) * 1000.0
        try:
            dev.dev.read(EP_IN, 64, timeout=5)
        except Exception:
            pass
        return {
            "ctrl": None,
            "val": None,
            "raw": None,
            "ms": dt_ms,
            "error": type(exc).__name__,
        }


def pct_to_raw(pct):
    pct = max(0.0, min(100.0, float(pct)))
    return int(round(pct * 1023.0 / 100.0))


def raw_to_pct(raw):
    return (float(raw) / 1023.0) * 100.0


def fmt_reply(tag, reply):
    if reply["error"]:
        return f"  [{tag}] SIN RESPUESTA  {reply['error']}  {reply['ms']:.0f} ms"
    mark = ""
    if tag.startswith("0x20") and reply["ctrl"] != 0x20:
        mark = "  <- la app tira este paquete"
    if tag.startswith("0x10") and reply["ctrl"] != 0x10:
        mark = "  <- primer byte distinto de 0x10"
    val = reply["val"]
    extra = ""
    if val is not None and 0 <= val <= 1023:
        extra = f"  posicion={val}/1023 ({raw_to_pct(val):.1f}%)"
    return (
        f"  [{tag}] bytes={reply['raw'].hex(' ')}  "
        f"ctrl=0x{reply['ctrl']:02X}  val={val}  {reply['ms']:.0f} ms"
        f"{extra}{mark}"
    )


def apply_ram(dev, cfg):
    ok = dev.apply_configuration(cfg, save_to_eeprom=False)
    # 0x21 SET_SPEED al máximo. Si alguien lo dejó en 0, el eje no arranca.
    speed = raw_xfer(dev, 0x21, 1023, timeout_ms=300)
    return ok, speed


def read_pos(dev, timeout_ms=400):
    reply = raw_xfer(dev, 0x10, 0, timeout_ms=timeout_ms)
    if reply["error"] or reply["val"] is None or not (0 <= reply["val"] <= 1023):
        return None, reply
    return int(reply["val"]), reply


def move_to(dev, raw, timeout_ms=400):
    reply = raw_xfer(dev, 0x20, raw, timeout_ms=timeout_ms)
    if reply["error"] or reply["val"] is None or not (0 <= reply["val"] <= 1023):
        return None, reply
    return int(reply["val"]), reply


def wait_settle(dev, target, timeout_s=2.5, tol=12):
    t0 = time.perf_counter()
    hist = []
    times = []
    misses = 0
    while True:
        pos, reply = read_pos(dev, timeout_ms=250)
        now = time.perf_counter() - t0
        if pos is None:
            misses += 1
        else:
            hist.append(pos)
            times.append(now)
        if now >= timeout_s:
            break
        if len(hist) >= 6:
            tail = hist[-6:]
            if max(abs(v - target) for v in tail) <= tol and (max(tail) - min(tail)) <= 4:
                break
        time.sleep(0.03)
    return hist, times, misses


def step_cost(hist, target, accuracy):
    if len(hist) < 3:
        return 1e6, {"overshoot": 999, "err": 999, "reversals": 99, "n": len(hist)}
    y = hist
    direction = 1 if target >= y[0] else -1
    overshoot = 0
    for v in y:
        beyond = (v - target) * direction
        if beyond > overshoot:
            overshoot = beyond
    err = abs(y[-1] - target)
    reversals = 0
    prev = 0
    for a, b in zip(y, y[1:]):
        d = b - a
        if abs(d) < 2:
            continue
        sign = 1 if d > 0 else -1
        if prev and sign != prev:
            reversals += 1
        prev = sign
    # Llegar importa más que la velocidad. Oscilar se castiga fuerte.
    cost = 8.0 * overshoot + 30.0 * reversals + 10.0 * max(0, err - accuracy) + 2.0 * err
    return cost, {
        "overshoot": overshoot,
        "err": err,
        "reversals": reversals,
        "n": len(y),
        "final": y[-1],
    }


def diagnose(dev, dwell_s=1.0):
    print("\n--- Diagnóstico, sin mover el objetivo ---")
    timeouts = 0
    odd = 0
    for i in range(8):
        reply = raw_xfer(dev, 0x10, 0, timeout_ms=400)
        print(fmt_reply(f"0x10 #{i + 1}", reply))
        if reply["error"]:
            timeouts += 1
        elif reply["ctrl"] != 0x10:
            odd += 1
        time.sleep(0.05)

    pos, reply = read_pos(dev)
    print(fmt_reply("0x10 actual", reply))
    if pos is None:
        print("  No hay una posición válida. No sigo al movimiento.")
        return False

    # Manda la posición en la que ya está. No debe recorrer carrera.
    reply = raw_xfer(dev, 0x20, pos, timeout_ms=400)
    print(fmt_reply("0x20 a la posición actual", reply))
    if reply["error"]:
        timeouts += 1
    elif reply["ctrl"] not in (0x20, 0x10):
        odd += 1

    time.sleep(dwell_s)
    print(f"\n  Timeouts: {timeouts}    respuestas con primer byte raro: {odd}")
    if timeouts >= 3:
        print("  Esto no se arregla con Kp o Kd.")
        print("  Revisa el cable mini-USB de ESA tarjeta, el puerto del hub y que la app esté cerrada.")
        return False
    if odd:
        print("  La tarjeta sí contesta, pero el primer byte no es el que la app espera.")
        print("  Por eso el registro dice 'eje 3 sin respuesta' aunque el pistón se mueva.")
    else:
        print("  Esta tarjeta contesta bien cuando se le habla sola.")
    return True


def tune_axis(dev, base_cfg):
    print("\n--- Sintonía del eje libre ---")
    print("El vástago tiene que estar suelto. El recorrido de prueba es de 25% a 55%.")
    ans = input("¿El eje 3 ya está desacoplado? [s/N]: ").strip().lower()
    if ans != "s":
        print("No muevo nada.")
        return None

    start, reply = read_pos(dev)
    if start is None:
        print(fmt_reply("lectura inicial", reply))
        return None
    print(f"  Posición inicial: {start}/1023 ({raw_to_pct(start):.1f}%)")

    low = pct_to_raw(25)
    high = pct_to_raw(55)

    cfg = dict(base_cfg)
    cfg["retract_limit"] = 0
    cfg["extend_limit"] = 1023
    cfg["max_pwm"] = 1023
    cfg["kp"] = 1
    cfg["movement_threshold"] = 3
    cfg["derivative_threshold"] = 3
    cfg["average_rc"] = 4
    cfg["stall_time"] = 10000

    # Ruido quieto, para no poner un accuracy más chico que el ADC.
    cfg["average_adc"] = 8
    cfg["accuracy"] = 8
    cfg["min_pwm"] = 80
    cfg["kd"] = 10
    cfg["pwm_threshold"] = 80
    apply_ram(dev, cfg)
    samples = []
    for _ in range(20):
        pos, _ = read_pos(dev, timeout_ms=250)
        if pos is not None:
            samples.append(pos)
        time.sleep(0.02)
    if len(samples) < 10:
        print("  Se perdieron lecturas en reposo. No sintonizo encima de un USB flojo.")
        return None
    noise = max(samples) - min(samples)
    accuracy = max(6, min(16, noise + 4))
    print(f"  Ruido en reposo: {noise} conteos. Accuracy de prueba: {accuracy}.")

    def go(raw):
        move_to(dev, raw)
        hist, _, misses = wait_settle(dev, raw, timeout_s=2.8, tol=max(8, accuracy))
        return hist, misses

    # Llévalo al 25% antes de medir.
    cfg["accuracy"] = accuracy
    apply_ram(dev, cfg)
    go(low)

    best = None
    trials = []
    for min_pwm in (90, 110, 130):
        for kd in (6, 10):
            for pwm_th in (60, 90):
                trial = dict(cfg)
                trial["min_pwm"] = min_pwm
                trial["kd"] = kd
                trial["pwm_threshold"] = pwm_th
                trial["accuracy"] = accuracy
                trial["average_adc"] = 6
                apply_ram(dev, trial)
                go(low)
                hist, misses = go(high)
                cost, info = step_cost(hist, high, accuracy)
                cost += 40.0 * misses
                trials.append((cost, trial, info, misses))
                print(
                    f"  min_pwm={min_pwm:3d}  kd={kd:2d}  pwm_th={pwm_th:3d}  "
                    f"err={info['err']:3d}  over={info['overshoot']:3.0f}  "
                    f"rev={info['reversals']}  misses={misses}  costo={cost:.0f}"
                )
                if best is None or cost < best[0]:
                    best = (cost, trial, info, misses)

    # Vuelve al punto donde estaba, con la mejor de esta pasada.
    winner = dict(best[1])
    apply_ram(dev, winner)
    go(start)

    print("\n  Mejor de esta pasada (solo RAM de esta tarjeta):")
    for key in ("accuracy", "min_pwm", "kd", "kp", "pwm_threshold", "average_adc"):
        print(f"    {key}: {winner[key]}")
    print(
        f"  error final del escalón: {best[2]['err']} conteos, "
        f"sobrepaso: {best[2]['overshoot']:.0f}, reversiones: {best[2]['reversals']}"
    )
    if best[3] > 2:
        print("  Hubo huecos de USB durante el escalón. Esa sintonía no es de fiar.")
    return winner


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eje", type=int, default=3, help="Eje 1..6. Por defecto 3.")
    args = parser.parse_args()
    axis = args.eje
    if not 1 <= axis <= 6:
        print("El eje tiene que estar entre 1 y 6.")
        return

    print("=" * 64)
    print(f"  SINTONÍA SOLO DEL EJE L{axis}")
    print("  Cierra la interfaz. Si sigue abierta, esta prueba no puede abrir el USB.")
    print("=" * 64)

    hub = LacUsbHubController()
    count, logs = hub.scan_and_open()
    for line in logs:
        print(" ", line)
    if count == 0:
        print("No se abrió ninguna LAC.")
        return

    dev = hub.devices_by_axis.get(axis)
    if dev is None:
        print(f"\nNo hay una tarjeta asignada a L{axis}. Estas son las que sí aparecen:")
        for info in hub.get_devices_summary():
            print(
                f"  USB #{info.raw_index}  serial={info.serial_num}  "
                f"eje L{info.assigned_axis}"
            )
        hub.close_all()
        return

    print(
        f"\nHablo solo con L{axis}: USB #{dev.raw_index}, serial {dev.serial_num}."
    )
    print("Los otros ejes no reciben 0x20.")

    params = load_parameters()
    cfg = dict(DEFAULT_LAC_CONFIG)
    if isinstance(params, dict) and isinstance(params.get("LAC_CONFIG"), dict):
        cfg.update(params["LAC_CONFIG"])

    while True:
        print("\n" + "-" * 64)
        print("  1) Diagnóstico de respuesta (no recorre)")
        print("  2) Mover solo este eje a un porcentaje")
        print("  3) Sintonizar este eje, desacoplado")
        print("  4) Subir a RAM la configuración que ya está en parameters.json")
        print("  0) Salir")
        op = input("Opción: ").strip()

        if op == "0":
            break
        elif op == "1":
            diagnose(dev)
        elif op == "2":
            try:
                pct = float(input("  Porcentaje 15 a 80: ").strip())
            except ValueError:
                print("  Número inválido.")
                continue
            if not 15.0 <= pct <= 80.0:
                print("  Fuera de 15..80. Así no lo llevo a tope con el vástago suelto.")
                continue
            raw = pct_to_raw(pct)
            fb, reply = move_to(dev, raw)
            print(fmt_reply("0x20", reply))
            time.sleep(0.6)
            pos, reply2 = read_pos(dev)
            print(fmt_reply("0x10 +0.6 s", reply2))
            if pos is not None:
                print(f"  Quedó en {raw_to_pct(pos):.1f}%  (pedido {pct:.1f}%)")
        elif op == "3":
            winner = tune_axis(dev, cfg)
            if winner is None:
                continue
            save = input("¿Escribir esta sintonía en parameters.json? Se la llevarán las seis. [s/N]: ").strip().lower()
            if save == "s":
                from config.parameters import save_lac_config
                save_lac_config(winner)
                print("  Guardada. La app la sube a las seis al conectar. EEPROM no se tocó.")
            else:
                print("  No se guardó. Sigue en la RAM de esta tarjeta hasta que la quites de la corriente.")
        elif op == "4":
            ok, speed = apply_ram(dev, cfg)
            print(f"  Configuración de archivo subida a RAM de L{axis}. ok={ok}")
            print(fmt_reply("0x21 velocidad máxima", speed))
        else:
            print("  Opción inválida.")

    hub.close_all()
    print("USB cerrado.")


if __name__ == "__main__":
    main()
