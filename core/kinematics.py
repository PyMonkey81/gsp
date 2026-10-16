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
from kinematics.inverse import inverse_kinematics as _raw_ik, TIK, PIK, home_kinematics
from kinematics.direct import (
    direct_kinematics as _raw_dk, CA, TSDK, TSVK, PDK, R_to_rpy
)
from kinematics.pose import pose_to_q as _raw_pose_to_q, rpy_to_R
from kinematics.jacobian import InvJac as _raw_inv_jac
from logic.spline import SimpleCubicSpline

import config.parameters as _params
from config.parameters import (
    Az, Bz, D, RT, ALPHA_0, L0, STROKE,
    length_to_percent, percent_to_length, q_to_extension, extension_to_q
)


def pose_to_q(x: float, y: float, z: float, roll: float, pitch: float, yaw: float, deg: bool = True):
    """
    Calculates actuator lengths for Cartesian pose.
    x, y, z in mm (offset relative to D; at 0,0,0 and 0,0,0 deg all actuators are at HOME = 0 mm extension).
    roll, pitch, yaw in degrees (if deg=True) or radians.
    """
    r = np.deg2rad(roll) if deg else float(roll)
    p = np.deg2rad(pitch) if deg else float(pitch)
    yw = np.deg2rad(yaw) if deg else float(yaw)
    return _raw_pose_to_q(x, y, z, r, p, yw)


def inverse_kinematics(y, rt=None, a0=None, D=None, deg: bool = True, clamp_hemisphere: bool = True):
    """
    Inverse kinematics for task space [alpha, beta] on the hemisphere of radius rt.
    alpha: azimuth angle.
    beta: elevation from horizon (0°..90°, where 90° = zenith).
    If deg=True, inputs are converted to radians.
    """
    y_arr = np.asarray(y, dtype=float)
    if deg:
        y_rad = np.deg2rad(y_arr[:2])
    else:
        y_rad = y_arr[:2]
    return _raw_ik(y_rad, rt=rt, a0=a0, D=D, clamp_hemisphere=clamp_hemisphere)


def direct_kinematics(
    q, qp=None, da0=None, R0=None, Az=None, Bz=None, D=None, rt=None, a0=None, deg: bool = True
):
    """
    Direct kinematics from actuator lengths q (and optional velocities qp).
    Returns task-space angles Y=[alpha, beta] (in degrees if deg=True, else radians),
    along with (da, R) or (Yp, da, R, Va, Omega).
    """
    az_use = _params.Az if Az is None else Az
    bz_use = _params.Bz if Bz is None else Bz
    d_use = _params.D if D is None else D
    rt_use = _params.RT if rt is None else rt
    a0_use = _params.ALPHA_0 if a0 is None else a0

    res = _raw_dk(q, qp=qp, da0=da0, R0=R0, Az=az_use, Bz=bz_use, D=d_use, rt=rt_use, a0=a0_use)
    if qp is None:
        Y_rad, da, R = res
        Y_out = np.rad2deg(Y_rad) if deg else Y_rad
        return Y_out, da, R
    else:
        Y_rad, Yp_rad, da, R, Va, Omega = res
        Y_out = np.rad2deg(Y_rad) if deg else Y_rad
        Yp_out = np.rad2deg(Yp_rad) if deg else Yp_rad
        return Y_out, Yp_out, da, R, Va, Omega


def InvJac(da: np.ndarray, R: np.ndarray, Az=None, Bz=None):
    """
    Calculates the 6x6 Inverse Jacobian matrix.
    """
    az_use = _params.Az if Az is None else Az
    bz_use = _params.Bz if Bz is None else Bz
    res = _raw_inv_jac(da, R, Az=az_use, Bz=bz_use)
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
