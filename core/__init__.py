"""Core package for kinematics and serial communication."""
from .kinematics import pose_to_q, inverse_kinematics, InvJac, calculate_singularity_metrics, SimpleCubicSpline
from .serial_manager import SerialManager
