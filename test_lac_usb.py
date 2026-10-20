"""
Script rápido de diagnóstico para probar 1 o más tarjetas Actuonix LAC por USB.
Ejecutar con:
    python test_lac_usb.py
"""

import time
from connection.lac_usb_driver import LacUsbHubController, LAC_VID, LAC_PID
from config.parameters import load_parameters, DEFAULT_LAC_CONFIG


def main():
    print("=" * 65)
    print("  DIAGNÓSTICO USB ACTUONIX LAC (VID=0x04D8, PID=0xFC5F)")
    print("=" * 65)

    hub = LacUsbHubController()
    count, logs = hub.scan_and_open()

    for line in logs:
        print(" ", line)

    if count == 0:
        print("\n[!] No se detectó ninguna tarjeta LAC abierta por PyUSB/mpusbapi.")
        print("    Posibles causas:")
        print("    1. Verifica que el cable Mini-USB sea de DATOS (no solo de carga).")
        print("    2. Si en el Administrador de Dispositivos aparece como 'Custom USB Device'")
        print("       (Microchip mchpusb.sys) y PyUSB no puede abrirla en 64 bits:")
        print("       -> Abre Zadig (https://zadig.akeo.ie/), selecciona la LAC (04D8:FC5F)")
        print("          y cambia el driver a 'WinUSB'.")
        return

    print(f"\n[OK] Se detectaron {count} tarjeta(s) LAC conectada(s):")
    for info in hub.get_devices_summary():
        pct = (info.last_raw_pos / 1023.0) * 100.0
        print(
            f"  -> USB #{info.raw_index} | Serial ID={info.serial_num} "
            f"| Asignada a: Actuador L{info.assigned_axis} "
            f"| Posición ADC actual: {info.last_raw_pos}/1023 ({pct:.1f}%)"
        )

    # Subir configuración de prueba en RAM
    params = load_parameters()
    cfg = dict(DEFAULT_LAC_CONFIG)
    if isinstance(params, dict) and "LAC_CONFIG" in params:
        cfg.update(params["LAC_CONFIG"])

    print("\nSubiendo configuración inicial (0x01..0x0F) a la(s) tarjeta(s)...")
    ok_cfg, cfg_logs = hub.upload_configuration_all(cfg, save_to_eeprom=False)
    for line in cfg_logs:
        print(" ", line)

    dev = hub.raw_devices[0]
    axis = dev.assigned_axis

    while True:
        print("\n" + "-" * 65)
        print(f"Tarjeta activa: USB #0 (Serial ID={dev.serial_num}, Eje L{axis})")
        print("Opciones de prueba:")
        print("  1) Leer posición actual (0x10 GET_FEEDBACK)")
        print("  2) Mover actuador a un porcentaje (0..100%) con 0x20 SET_POSITION")
        print("  3) Asignar/Cambiar ID de actuador (1..6) en EEPROM (0x50 SET_SERIAL_NUM)")
        print("  4) Restaurar valores de fábrica y potenciómetros manuales (0xFF RESET)")
        print("  0) Salir")
        op = input("Selecciona una opción: ").strip()

        if op == "0":
            break
        elif op == "1":
            raw = dev.get_feedback_raw()
            if raw is not None:
                print(f"  [FEEDBACK] ADC = {raw}/1023 ({raw * 100.0 / 1023.0:.2f}%)")
            else:
                print("  [ERROR] Sin respuesta de la tarjeta LAC.")
        elif op == "2":
            try:
                pct = float(input("  Ingresa porcentaje destino (0 a 100): ").strip())
                raw_target = int(round(max(0.0, min(100.0, pct)) * 1023.0 / 100.0))
                fb_raw = dev.set_position_raw(raw_target)
                print(f"  [SET_POSITION] Enviado={raw_target}/1023 | Feedback inmediato={fb_raw}/1023")
                time.sleep(0.5)
                fb_after = dev.get_feedback_raw()
                if fb_after is not None:
                    print(f"  [FEEDBACK +0.5s] ADC={fb_after}/1023 ({fb_after * 100.0 / 1023.0:.2f}%)")
            except ValueError:
                print("  Valor inválido.")
        elif op == "3":
            try:
                new_id = int(input("  Ingresa nuevo ID de Actuador (1 a 6): ").strip())
                ok, msg = hub.assign_device_serial(dev.raw_index, new_id)
                print(" ", msg)
                axis = dev.assigned_axis
            except ValueError:
                print("  Valor inválido.")
        elif op == "4":
            if dev.reset_factory_defaults():
                print("  [RESET] Comando 0xFF enviado correctamente.")
            else:
                print("  [ERROR] Falló envío de 0xFF.")

    hub.close_all()
    print("Conexión USB cerrada.")


if __name__ == "__main__":
    main()
