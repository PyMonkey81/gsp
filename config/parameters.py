"""
Platform geometric and operational parameters for Gough-Stewart 6-DOF Robot.
Prototype CAD parameters from original project (Actuonix L16-100 actuators).
"""

import json
from pathlib import Path
import numpy as np

# Carrera real del Actuonix L16-100 (mm)
STROKE = 100.0

# Longitud muerta ancla a ancla con el émbolo retraído (mm)
L0 = 249.4

# Rango físico completo: HOME es 0% (todo retraído)
ACTUATOR_MIN = L0                 # 0% retraído (~249.4 mm)
ACTUATOR_MAX = L0 + STROKE        # 100% (~349.4 mm)
ACTUATOR_HOME_PERCENT = 0.0       # HOME = 0% (retraído, cenit)

# Aliases para compatibilidad UI
Q_MIN = ACTUATOR_MIN
Q_MAX = ACTUATOR_MAX
Q_NOMINAL = 247.4
SAFE_MARGIN = 5.0
Q_SAFE_MIN = Q_MIN + SAFE_MARGIN
Q_SAFE_MAX = Q_MAX - SAFE_MARGIN

# ====================== Geometría (escala prototipo, mm) ======================
# Az: puntos de anclaje en marco plataforma móvil
# Bz: puntos de anclaje en marco base fijo (Pin01 a Pin06)
Az = np.array([
    [ 46.95,  39.63, -44.33],     # eje 1 (Pin01)
    [ 10.84,  60.48, -44.33],     # eje 2
    [-57.79,  20.85, -44.33],     # eje 3
    [-57.79, -20.85, -44.33],     # eje 4
    [ 10.84, -60.48, -44.33],     # eje 5
    [ 46.95, -39.63, -44.33],     # eje 6
], dtype=np.float64)

Bz = np.array([
    [ 87.79,  20.85, 44.32],      # eje 1 (Pin01)
    [-25.84,  86.46, 44.32],      # eje 2
    [-61.95,  65.61, 44.32],      # eje 3
    [-61.95, -65.61, 44.32],      # eje 4
    [-25.84, -86.46, 44.32],      # eje 5
    [ 87.79, -20.85, 44.32],      # eje 6
], dtype=np.float64)

# Compatibilidad con visor 3D
BASE_JOINTS = Bz
PLATFORM_JOINTS = Az

# ====================== Parámetros de la plataforma (en mm) ======================
ALPHA_0 = 0.0
D = np.array([0.0, 0.0, 331.94], dtype=np.float64)  # Centro de agujero
RT = 0.0                                            # Antena offset (0 para prototipo)
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


def length_to_percent(q_mm: np.ndarray, l0: float = L0, stroke: float = STROKE) -> np.ndarray:
    """
    Largo total ancla a ancla (mm, salida de PIK) a porcentaje de carrera (0 a 100%).
    pct = sat((q - L0) / carrera * 100, 0, 100).
    """
    percent = (np.asarray(q_mm, dtype=float) - l0) / stroke * 100.0
    return np.clip(percent, 0.0, 100.0)


def percent_to_length(percent: np.ndarray, l0: float = L0, stroke: float = STROKE) -> np.ndarray:
    """
    Convierte porcentaje de carrera a longitud en mm.
    q_mm = l0 + (percent / 100.0) * stroke
    """
    return l0 + (np.asarray(percent, dtype=float) / 100.0) * stroke


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
