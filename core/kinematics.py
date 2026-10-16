"""
Kinematics and Workspace Analysis for 6-DOF Gough-Stewart Platform.

Includes:
- pose_to_q(x, y, z, roll, pitch, yaw) -> q_mm, da, R
- inverse_kinematics(y=[alpha, beta]) -> q_mm, da, R
- InvJac(da, R) -> 6x6 Inverse Jacobian Matrix
- calculate_singularity_metrics(J_inv) -> dict
- SimpleCubicSpline for smooth trajectory generation
"""

from typing import Tuple, Union, Optional
import numpy as np

try:
    from config.parameters import BASE_JOINTS, PLATFORM_JOINTS, Z_HOME, Q_MIN, Q_MAX, CHAR_LENGTH
except ImportError:
    BASE_JOINTS = np.zeros((6, 3))
    PLATFORM_JOINTS = np.zeros((6, 3))
    Z_HOME = 300.0
    Q_MIN = 220.0
    Q_MAX = 420.0
    CHAR_LENGTH = 150.0


def euler_to_rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Computes 3x3 rotation matrix using Z-Y-X convention (Yaw -> Pitch -> Roll).
    Angles in degrees.
    """
    phi = np.deg2rad(roll_deg)
    theta = np.deg2rad(pitch_deg)
    psi = np.deg2rad(yaw_deg)

    # Elementary rotation matrices
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(phi), -np.sin(phi)],
        [0.0, np.sin(phi), np.cos(phi)]
    ], dtype=np.float64)

    ry = np.array([
        [np.cos(theta), 0.0, np.sin(theta)],
        [0.0, 1.0, 0.0],
        [-np.sin(theta), 0.0, np.cos(theta)]
    ], dtype=np.float64)

    rz = np.array([
        [np.cos(psi), -np.psi if False else -np.sin(psi), 0.0],
        [np.sin(psi), np.cos(psi), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    # R = Rz(psi) * Ry(theta) * Rx(phi)
    return rz @ ry @ rx


def pose_to_q(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
    base_joints: Optional[np.ndarray] = None,
    platform_joints: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute Inverse Kinematics for Cartesian 6-DOF Pose.
    
    Parameters:
        x, y, z: Mobile platform position in world frame (mm).
        roll, pitch, yaw: Platform orientation angles in degrees.
        base_joints: (6, 3) array of fixed base anchor coordinates (mm).
        platform_joints: (6, 3) array of mobile platform coordinates in local frame (mm).

    Returns:
        q_mm: np.ndarray shape (6,) actuator lengths in mm.
        da: np.ndarray shape (3,) translation vector [x, y, z].
        R: np.ndarray shape (3, 3) rotation matrix.
    """
    if base_joints is None:
        base_joints = BASE_JOINTS
    if platform_joints is None:
        platform_joints = PLATFORM_JOINTS

    da = np.array([float(x), float(y), float(z)], dtype=np.float64)
    R = euler_to_rotation_matrix(float(roll), float(pitch), float(yaw))

    # Transformed platform anchor locations in base frame: A_i = da + R * P_i
    ai = da + (platform_joints @ R.T)

    # Leg vectors l_i = A_i - B_i
    leg_vectors = ai - base_joints

    # Actuator lengths q_i = ||l_i||
    q_mm = np.linalg.norm(leg_vectors, axis=1)

    return q_mm, da, R


def inverse_kinematics(
    y: Union[list, np.ndarray],
    z_nominal: float = Z_HOME,
    base_joints: Optional[np.ndarray] = None,
    platform_joints: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Inverse Kinematics for Track Mode (Task Space pointing angles).
    
    Parameters:
        y: Task space vector [alpha, beta] or [alpha, beta, z], where:
           - alpha: Azimuth angle in degrees (-180° to +180°)
           - beta: Elevation / Tilt angle from vertical in degrees (0° to 35°)
           - (optional) z: Operating height in mm.
        z_nominal: Platform vertical center if z is not provided in y.

    Returns:
        q_mm: (6,) Actuator lengths in mm.
        da: (3,) Platform translation vector.
        R: (3, 3) Platform rotation matrix.
    """
    alpha = float(y[0])
    beta = float(y[1])
    z = float(y[2]) if len(y) > 2 else float(z_nominal)

    # Tilt direction: tilt beta about axis perpendicular to azimuth
    alpha_rad = np.deg2rad(alpha)
    roll = beta * np.sin(alpha_rad)
    pitch = beta * np.cos(alpha_rad)
    yaw = 0.0

    return pose_to_q(
        x=0.0,
        y=0.0,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        base_joints=base_joints,
        platform_joints=platform_joints
    )


def InvJac(
    da: np.ndarray,
    R: np.ndarray,
    base_joints: Optional[np.ndarray] = None,
    platform_joints: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Calculate the 6x6 Inverse Jacobian Matrix (J_inv) of the Gough-Stewart platform.
    
    Relates platform operational space velocity [v; omega] to actuator stroke rates dq:
        dq = J_inv @ [v; omega]
        
    Row i is:
        J_inv[i, :] = [u_i.T, (r_i x u_i).T]
    where:
        u_i = (A_i - B_i) / ||A_i - B_i|| (unit vector along actuator)
        r_i = R @ P_i (vector from platform center to platform joint anchor)

    Returns:
        J_inv: np.ndarray shape (6, 6)
    """
    if base_joints is None:
        base_joints = BASE_JOINTS
    if platform_joints is None:
        platform_joints = PLATFORM_JOINTS

    da = np.asarray(da, dtype=np.float64)
    R = np.asarray(R, dtype=np.float64)

    J_inv = np.zeros((6, 6), dtype=np.float64)

    for i in range(6):
        pi = platform_joints[i]
        bi = base_joints[i]
        
        # r_i in base frame
        ri = R @ pi
        ai = da + ri
        li = ai - bi
        length = np.linalg.norm(li)
        
        if length < 1e-6:
            ui = np.array([0.0, 0.0, 1.0])
        else:
            ui = li / length
            
        cross_term = np.cross(ri, ui)
        
        J_inv[i, 0:3] = ui
        J_inv[i, 3:6] = cross_term

    return J_inv


def calculate_singularity_metrics(J_inv: np.ndarray, char_len: float = CHAR_LENGTH) -> dict:
    """
    Calculate singularity indices and manipulability metrics from J_inv.
    Uses dimensionally normalized Jacobian to compute physically meaningful condition number.
    
    Returns dict:
        - det: determinant of J_inv
        - cond: normalized condition number (1.0 = isotropic, > 30 = ill-conditioned)
        - manipulability: Yoshikawa manipulability index sqrt(det(J_inv * J_inv.T))
        - min_sv: minimum singular value of normalized Jacobian
        - status: Human-readable safety string
        - status_code: 'OPTIMAL', 'SAFE', 'WARNING', or 'CRITICAL'
    """
    try:
        det_val = float(np.linalg.det(J_inv))
        
        # Normalize angular columns (last 3 columns) by characteristic length
        J_norm = J_inv.copy()
        J_norm[:, 3:6] /= max(char_len, 1e-3)
        
        sv = np.linalg.svd(J_norm, compute_uv=False)
        min_sv = float(sv[-1]) if len(sv) > 0 else 0.0
        max_sv = float(sv[0]) if len(sv) > 0 else 1.0
        cond_val = float(max_sv / (min_sv + 1e-12))
        manip = float(np.abs(det_val))

        if min_sv < 0.15 or cond_val > 35.0:
            status = "CRÍTICO (Cerca de Singularidad)"
            status_code = "CRITICAL"
        elif min_sv < 0.35 or cond_val > 15.0:
            status = "PRECAUCIÓN (Destreza Reducida)"
            status_code = "WARNING"
        elif cond_val > 6.0:
            status = "SEGURO (Operación Estable)"
            status_code = "SAFE"
        else:
            status = "ÓPTIMO (Alta Isotropía)"
            status_code = "OPTIMAL"

        return {
            "det": det_val,
            "cond": cond_val,
            "manipulability": manip,
            "min_sv": min_sv,
            "status": status,
            "status_code": status_code,
            "singular_values": sv
        }
    except Exception as e:
        return {
            "det": 0.0,
            "cond": 999.9,
            "manipulability": 0.0,
            "min_sv": 0.0,
            "status": f"ERROR: {e}",
            "status_code": "ERROR",
            "singular_values": np.zeros(6)
        }


class SimpleCubicSpline:
    """
    Cubic Spline Trajectory Interpolator.
    Supports smooth interpolation over time waypoints for multi-dimensional trajectories.
    """
    def __init__(self, t: Union[list, np.ndarray], y: Union[list, np.ndarray]):
        self.t = np.asarray(t, dtype=np.float64)
        self.y = np.asarray(y, dtype=np.float64)
        
        if len(self.t) < 2:
            raise ValueError("SimpleCubicSpline requires at least 2 time waypoints.")
        if len(self.t) != len(self.y):
            raise ValueError("t and y arrays must have identical length.")
            
        diffs = np.diff(self.t)
        if np.any(diffs <= 0):
            raise ValueError("Waypoints time t must be strictly monotonically increasing.")

        self._scipy_spline = None
        try:
            from scipy.interpolate import CubicSpline
            self._scipy_spline = CubicSpline(self.t, self.y, bc_type='clamped')
        except Exception:
            self._scipy_spline = None

    def __call__(self, t_query: Union[float, np.ndarray]) -> np.ndarray:
        return self.evaluate(t_query)

    def evaluate(self, t_query: Union[float, np.ndarray]) -> np.ndarray:
        """
        Evaluate spline interpolation at time t_query. Clamps outside bounds [t_0, t_end].
        """
        is_scalar = np.isscalar(t_query)
        t_arr = np.atleast_1d(t_query).astype(np.float64)
        t_clamped = np.clip(t_arr, self.t[0], self.t[-1])

        if self._scipy_spline is not None:
            out = self._scipy_spline(t_clamped)
            return out[0] if is_scalar else out

        # Pure-Python cubic hermite fallback
        results = []
        for t_val in t_clamped:
            idx = np.searchsorted(self.t, t_val) - 1
            idx = max(0, min(idx, len(self.t) - 2))
            
            t0, t1 = self.t[idx], self.t[idx + 1]
            y0, y1 = self.y[idx], self.y[idx + 1]
            dt = t1 - t0
            u = (t_val - t0) / dt if dt > 1e-12 else 0.0
            
            # Smooth Hermite S-curve blending: 3*u^2 - 2*u^3
            h00 = 2 * u**3 - 3 * u**2 + 1
            h01 = -2 * u**3 + 3 * u**2
            val = h00 * y0 + h01 * y1
            results.append(val)

        res = np.array(results)
        return res[0] if is_scalar else res

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])
