"""
Script de Sintonización Automática (Auto-Tuning PD + Filtro ADC) para tarjetas Actuonix LAC por USB.
Mide el ruido del ADC interno de 10 bits (0x10 GET_FEEDBACK), identifica la zona muerta / fricción
estática (min_pwm), la velocidad máxima e inercia de frenado, y optimiza en lazo cerrado:
  - Accuracy (0x01)
  - PWM Threshold (0x06)
  - Derivative Threshold (0x07)
  - PWM Minimum (0x0B)
  - Proportional Gain Kp (0x0C)
  - Derivative Gain Kd (0x0D)
  - Average ADC (0x0F)

Guarda automáticamente los parámetros óptimos en config/parameters.json (LAC_CONFIG).

Ejecutar con:
    python autotune_lac.py
"""

import argparse
from config.parameters import load_parameters, DEFAULT_LAC_CONFIG
from connection.lac_usb_driver import LacUsbHubController
from connection.lac_autotuner import LacAutoTuner


def main():
    parser = argparse.ArgumentParser(
        description="Auto-Tuning PD en lazo cerrado para tarjetas Actuonix LAC por USB (10-bit ADC)."
    )
    parser.add_argument(
        "--eeprom",
        action="store_true",
        help="Además de guardar en parameters.json y aplicar en RAM, graba en la EEPROM de las LAC (0x30).",
    )
    args = parser.parse_args()

    print("=" * 70)
    print("  AUTO-SINTONIZACIÓN PD ACTUONIX LAC POR USB (ADC 10-BIT FEEDBACK)")
    print("=" * 70)

    hub = LacUsbHubController()
    count, scan_logs = hub.scan_and_open()
    for line in scan_logs:
        print(" ", line)

    if count == 0:
        print("\n[!] No se detectó ninguna tarjeta Actuonix LAC conectada por USB (VID=0x04D8, PID=0xFC5F).")
        print("    Conecte el Hub USB o la tarjeta LAC y vuelva a intentarlo.")
        return

    params = load_parameters()
    base_cfg = dict(DEFAULT_LAC_CONFIG)
    if isinstance(params, dict) and "LAC_CONFIG" in params:
        base_cfg.update(params["LAC_CONFIG"])

    tuner = LacAutoTuner(
        hub=hub,
        base_cfg=base_cfg,
        log_cb=lambda msg: print(msg),
    )

    try:
        ok, tuned_cfg, _ = tuner.run_autotune(save_to_eeprom=args.eeprom)
        if ok:
            print("\n[OK] Los nuevos parámetros han sido guardados en config/parameters.json")
            if args.eeprom:
                print("[OK] También fueron grabados en la EEPROM de la(s) tarjeta(s) LAC (comando 0x30).")
    finally:
        hub.close_all()
        print("Conexión USB cerrada.")


if __name__ == "__main__":
    main()
