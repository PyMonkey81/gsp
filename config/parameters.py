"""
Platform geometric and operational parameters for Gough-Stewart 6-DOF Robot.
Prototype CAD parameters from original project (Actuonix L16-100 actuators).
"""

import json
from pathlib import Path
import numpy as np

# Carrera real del Actuonix L16-100 (mm)
STROKE = 100.0

# Longitud muerta ancla a ancla con el émbolo retraído en HOME (mm)
L0 = 247.41

# Rango físico completo: HOME es 0% (todo retraído = 0 mm de extensión)
ACTUATOR_MIN = L0                 # 0% retraído (247.41 mm ancla-a-ancla, 0.0 mm extensión)
ACTUATOR_MAX = L0 + STROKE        # 100% (347.41 mm ancla-a-ancla, 100.0 mm extensión)
ACTUATOR_HOME_PERCENT = 0.0       # HOME = 0% (retraído, cenit)

# Aliases para compatibilidad UI (en longitud total ancla-a-ancla mm)
Q_MIN = ACTUATOR_MIN
Q_MAX = ACTUATOR_MAX
Q_NOMINAL = ACTUATOR_MIN          # HOME nominal = todos retraídos (0 mm extensión)
SAFE_MARGIN = 5.0
Q_SAFE_MIN = Q_MIN + SAFE_MARGIN
Q_SAFE_MAX = Q_MAX - SAFE_MARGIN

# Límites en extensión efectiva q_ef = q - L0 (mm): HOME = 0.0 mm, MAX = STROKE (100.0 mm)
EXT_HOME = 0.0
EXT_MIN = 0.0
EXT_MAX = STROKE

# ====================== Geometría (escala prototipo, mm) ======================
# Az: puntos de anclaje en marco plataforma móvil
# Bz: puntos de anclaje en marco base fijo (Pin01 a Pin06)
Az = np.array([
    [ 19.09,  23.87, -35.24],     # eje 1 (Pin01)
    [ 11.13,  28.46, -35.24],     # eje 2
    [-30.21,   4.60, -35.24],     # eje 3
    [-30.21,  -4.60, -35.24],     # eje 4
    [ 11.13, -28.46, -35.24],     # eje 5
    [ 19.09, -23.87, -35.24],     # eje 6
], dtype=np.float64)

Bz = np.array([
    [ 62.32,   9.84, 35.70],      # eje 1 (Pin01)
    [-22.64,  58.89, 35.70],      # eje 2
    [-39.68,  49.05, 35.70],      # eje 3
    [-39.68, -49.05, 35.70],      # eje 4
    [-22.64, -58.89, 35.70],      # eje 5
    [ 62.32,  -9.84, 35.70],      # eje 6
], dtype=np.float64)

# Compatibilidad con visor 3D
BASE_JOINTS = Bz
PLATFORM_JOINTS = Az

# ====================== Parámetros de la plataforma (en mm) ======================
ALPHA_0 = 0.0
D = np.array([0.0, 0.0, 314.13], dtype=np.float64)  # Centro de agujero (HOME: da = D, R = I)
RT = 85.0                                           # Radio de la semiesfera virtual (da = D + rt * R[:, 2])
Z_HOME = float(D[2])

# ====================== Controlador Sliding-PI ======================
ALPHA_PI = 150.0
KP_S = 15000.0
KI_S = 300.0

# ====================== Trayectoria / Tiempos ======================
T_HOME_END = 60.0
T_TRACKING_END = 653.0
DT = 0.001
FILTER_WN = 3.4

# AUTO sequence timing: HOME → APPROACH → TRACK → RETURN → HOME
APPROACH_S = 3.0
RETURN_S = 3.0
DEMO_DURATION = 60.0

# Puntos de trayectoria por defecto (TRAJ_POINTS en grados: [alpha, beta])
# beta es elevación desde el horizonte (LMT: 10.10, 45.60, 10.00)
TRAJ_POINTS = [
    [9.9, 10.1],
    [84.5, 45.6],
    [158.5, 10.0]
]

# ====================== Modo CARTESIANO (Límites) ======================
ALPHA_BETA_LIMIT_DEG = 90.0

LIMITS_X = (-30.0, 30.0)
LIMITS_Y = (-30.0, 30.0)
LIMITS_Z = (-40.0, 40.0)  # Offset respecto a D[2]=331.94 mm
LIMITS_ROLL = (-15.0, 15.0)
LIMITS_PITCH = (-15.0, 15.0)
LIMITS_YAW = (-20.0, 20.0)

LIMITS_ALPHA = (-180.0, 180.0)
LIMITS_BETA = (0.0, 90.0)  # Elevación desde horizonte (90° = Cenit/Home)

# Communication parameters
DEFAULT_BAUDRATE = 115200
AVAILABLE_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
TELEMETRY_REFRESH_HZ = 30
SERIAL_STREAM_HZ = 30


def q_to_extension(q_mm: np.ndarray, l0: float = None) -> np.ndarray:
    """
    Convierte longitud total ancla-a-ancla q (mm) a extensión efectiva del actuador q_ef = q - L0 (mm).
    En HOME (actuadores retraídos), q = L0 => q_ef = 0.0 mm.
    Equivale al bloque Subtract2 (qd_ef = q - L0) de Simulink.
    """
    ref_l0 = L0 if l0 is None else float(l0)
    return np.asarray(q_mm, dtype=float) - ref_l0


def extension_to_q(ext_mm: np.ndarray, l0: float = None) -> np.ndarray:
    """
    Convierte extensión efectiva q_ef (mm) a longitud total ancla-a-ancla q = L0 + q_ef (mm).
    En HOME (q_ef = 0 mm), devuelve q = L0.
    """
    ref_l0 = L0 if l0 is None else float(l0)
    return ref_l0 + np.asarray(ext_mm, dtype=float)


def length_to_percent(q_mm: np.ndarray, l0: float = None, stroke: float = None) -> np.ndarray:
    """
    Largo total ancla a ancla (mm, salida de PIK) a porcentaje de carrera (0 a 100%).
    pct = sat((q - L0) / carrera * 100, 0, 100).
    """
    ref_l0 = L0 if l0 is None else float(l0)
    ref_stroke = STROKE if stroke is None else float(stroke)
    percent = (np.asarray(q_mm, dtype=float) - ref_l0) / ref_stroke * 100.0
    return np.clip(percent, 0.0, 100.0)


def percent_to_length(percent: np.ndarray, l0: float = None, stroke: float = None) -> np.ndarray:
    """
    Convierte porcentaje de carrera a longitud en mm.
    q_mm = l0 + (percent / 100.0) * stroke
    """
    ref_l0 = L0 if l0 is None else float(l0)
    ref_stroke = STROKE if stroke is None else float(stroke)
    return ref_l0 + (np.asarray(percent, dtype=float) / 100.0) * ref_stroke


def load_parameters() -> dict:
    """Carga parameters.json si existe o devuelve los valores por defecto."""
    json_path = Path(__file__).resolve().parent / "parameters.json"
    if json_path.exists():
        try:
            with open(json_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
                return data
        except Exception:
            pass
    return {
        "L0": L0, "STROKE": STROKE, "ACTUATOR_HOME_PERCENT": ACTUATOR_HOME_PERCENT,
        "ALPHA_0": ALPHA_0, "D": D.tolist(), "RT": RT,
        "ALPHA_PI": ALPHA_PI, "KP_S": KP_S, "KI_S": KI_S,
        "T_HOME_END": T_HOME_END, "T_TRACKING_END": T_TRACKING_END,
        "DEMO_DURATION": DEMO_DURATION, "DT": DT, "FILTER_WN": FILTER_WN,
        "TRAJ_POINTS": TRAJ_POINTS,
        "Az": Az.tolist(), "Bz": Bz.tolist()
    }
