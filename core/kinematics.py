"""
Kinematics compatibility module for 6-DOF Gough-Stewart Platform.
Wraps authentic functions from kinematics package:
- pose_to_q(x, y, z, roll, pitch, yaw) -> q_mm, da, R
- inverse_kinematics(y=[alpha, beta]) -> q_mm, da, R
- InvJac(da, R) -> 6x6 Inverse Jacobian Matrix
- calculate_singularity_metrics(J_inv) -> dict
- SimpleCubicSpline
"""

import numpy as np

# Import authentic kinematics from kinematics package
from kinematics.inverse import inverse_kinematics as _raw_ik, TIK, PIK
from kinematics.pose import pose_to_q as _raw_pose_to_q, rpy_to_R
from kinematics.jacobian import InvJac as _raw_inv_jac
from logic.spline import SimpleCubicSpline

from config.parameters import Az, Bz, D, RT, ALPHA_0, L0, STROKE, length_to_percent


def pose_to_q(x: float, y: float, z: float, roll: float, pitch: float, yaw: float, deg: bool = True):
    """
    Calculates actuator lengths for Cartesian pose.
    x, y, z in mm.
    roll, pitch, yaw in degrees (if deg=True) or radians.
    """
    r = np.deg2rad(roll) if deg else float(roll)
    p = np.deg2rad(pitch) if deg else float(pitch)
    yw = np.deg2rad(yaw) if deg else float(yaw)
    return _raw_pose_to_q(x, y, z, r, p, yw)


def inverse_kinematics(y, rt=RT, a0=ALPHA_0, D=D, deg: bool = True):
    """
    Inverse kinematics for task space [alpha, beta].
    alpha: azimuth angle.
    beta: elevation from horizon (90° = zenith/home).
    If deg=True, inputs are converted to radians.
    """
    y_arr = np.asarray(y, dtype=float)
    if deg:
        y_rad = np.deg2rad(y_arr[:2])
    else:
        y_rad = y_arr[:2]
    return _raw_ik(y_rad, rt=rt, a0=a0, D=D)


def InvJac(da: np.ndarray, R: np.ndarray, Az=Az, Bz=Bz):
    """
    Calculates the 6x6 Inverse Jacobian matrix.
    """
    res = _raw_inv_jac(da, R, Az=Az, Bz=Bz)
    if isinstance(res, tuple):
        return res[0]  # Return 6x6 matrix A
    return res


def calculate_singularity_metrics(J_inv: np.ndarray, char_len: float = 100.0) -> dict:
    """
    Calculate manipulability and condition number from J_inv.
    """
    try:
        det_val = float(np.linalg.det(J_inv))
        J_norm = J_inv.copy()
        J_norm[:, 3:6] /= max(char_len, 1e-3)
        sv = np.linalg.svd(J_norm, compute_uv=False)
        min_sv = float(sv[-1]) if len(sv) > 0 else 0.0
        max_sv = float(sv[0]) if len(sv) > 0 else 1.0
        cond_val = float(max_sv / (min_sv + 1e-12))
        manip = float(np.abs(det_val))

        if min_sv < 0.10 or cond_val > 40.0:
            status = "CRÍTICO (Cerca de Singularidad)"
            status_code = "CRITICAL"
        elif min_sv < 0.25 or cond_val > 20.0:
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
