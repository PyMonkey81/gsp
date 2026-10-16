"""
Platform geometric and operational parameters for Gough-Stewart 6-DOF Robot.
"""
import numpy as np

# Base geometry parameters (mm)
BASE_RADIUS = 250.0  # Radius of fixed base circle
BASE_PAIR_SEPARATION_DEG = 30.0  # Angular separation of joint pairs on base

# Platform geometry parameters (mm)
PLATFORM_RADIUS = 150.0  # Radius of mobile platform circle
PLATFORM_PAIR_SEPARATION_DEG = 30.0  # Angular separation of joint pairs on platform

# Characteristic length for dimensionless Jacobian conditioning
CHAR_LENGTH = PLATFORM_RADIUS

# Nominal operational height (mm)
Z_HOME = 300.0

# Actuator mechanical limits (mm)
Q_MIN = 220.0       # Minimum actuator extension
Q_MAX = 420.0       # Maximum actuator extension
Q_NOMINAL = 332.0   # Actuator length at home position
STROKE = Q_MAX - Q_MIN
SAFE_MARGIN = 12.0   # Safe stroke margin in mm
Q_SAFE_MIN = Q_MIN + SAFE_MARGIN
Q_SAFE_MAX = Q_MAX - SAFE_MARGIN

# Workspace limits for Cartesian mode
LIMITS_X = (-100.0, 100.0)      # mm
LIMITS_Y = (-100.0, 100.0)      # mm
LIMITS_Z = (220.0, 380.0)       # mm
LIMITS_ROLL = (-25.0, 25.0)     # degrees
LIMITS_PITCH = (-25.0, 25.0)    # degrees
LIMITS_YAW = (-35.0, 35.0)      # degrees

# Workspace limits for Task / Track mode (Alpha/Beta pointing angles)
LIMITS_ALPHA = (-180.0, 180.0)  # degrees (azimuth)
LIMITS_BETA = (0.0, 35.0)       # degrees (elevation / tilt from vertical)

# Communication parameters
DEFAULT_BAUDRATE = 115200
AVAILABLE_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
TELEMETRY_REFRESH_HZ = 30
SERIAL_STREAM_HZ = 30

# Sliding-PI Controller parameters
ALPHA_PI = 5.0
KP_S = 12.0
KI_S = 0.5
DT = 1.0 / 30.0


def generate_base_joints(radius: float = BASE_RADIUS, pair_deg: float = BASE_PAIR_SEPARATION_DEG) -> np.ndarray:
    """
    Generate 6 anchor coordinates for the fixed base (z = 0).
    Pairs are centered at 0°, 120°, and 240°.
    Returns: np.ndarray shape (6, 3) in mm.
    """
    half_rad = np.deg2rad(pair_deg / 2.0)
    angles = np.array([
        -half_rad,               half_rad,
        (2*np.pi/3) - half_rad,  (2*np.pi/3) + half_rad,
        (4*np.pi/3) - half_rad,  (4*np.pi/3) + half_rad
    ])
    bx = radius * np.cos(angles)
    by = radius * np.sin(angles)
    bz = np.zeros(6)
    return np.column_stack([bx, by, bz])

def generate_platform_joints(radius: float = PLATFORM_RADIUS, pair_deg: float = PLATFORM_PAIR_SEPARATION_DEG) -> np.ndarray:
    """
    Generate 6 anchor coordinates for the mobile platform in local frame {P}.
    Interleaved pairs create a triangular truss configuration for optimal rigidity.
    Returns: np.ndarray shape (6, 3) in mm.
    """
    half_rad = np.deg2rad(pair_deg / 2.0)
    angles = np.array([
        (5*np.pi/3) + half_rad,  (np.pi/3) - half_rad,
        (np.pi/3) + half_rad,    np.pi - half_rad,
        np.pi + half_rad,        (5*np.pi/3) - half_rad
    ])
    px = radius * np.cos(angles)
    py = radius * np.sin(angles)
    pz = np.zeros(6)
    return np.column_stack([px, py, pz])

# Precalculated nominal joint anchor locations in base frame and local platform frame
BASE_JOINTS = generate_base_joints()
PLATFORM_JOINTS = generate_platform_joints()
