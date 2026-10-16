# kinematics/pose.py
"""Helper de pose cartesiana (X, Y, Z, Roll, Pitch, Yaw) para el modo CARTESIANO.

No modifica ni reemplaza TIK/PIK de kinematics/inverse.py: solo arma
(da, R) a partir de una pose cartesiana y llama PIK, igual que
inverse_kinematics(y) arma (da, R) a partir de (alpha, beta) vía TIK.
"""
import numpy as np

from kinematics.inverse import PIK
from config.parameters import D


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Matriz de rotación R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Ángulos en radianes."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])

    return Rz @ Ry @ Rx


def pose_to_q(x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
    """Pose cartesiana (mm, rad) -> longitudes de actuador en mm, vía PIK.

    da = D + [x, y, z]   (mismo convenio de traslación que usa TIK: origen de plataforma
                          respecto a la base, desplazado por D)
    R  = rpy_to_R(roll, pitch, yaw)
    
    Returns: q_mm (largo total ancla a ancla, sin restar L0), da (mm), R (3x3)
    """
    da = D + np.array([x, y, z])
    R = rpy_to_R(roll, pitch, yaw)
    q_mm = PIK(da, R)
    return q_mm, da, R
