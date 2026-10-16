# logic/spline.py
import numpy as np
from scipy.interpolate import CubicSpline as ScipyCubicSpline


class SimpleCubicSpline:
    """
    Wrapper de scipy.interpolate.CubicSpline
    con la misma interfaz que tenías antes.
    """
    def __init__(self, x, y, bc_type='natural'):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self._spline = ScipyCubicSpline(self.x, self.y, bc_type=bc_type)

    def __call__(self, x_new, der=0):
        """
        der = 0 → valor
        der = 1 → primera derivada
        """
        return self._spline(x_new, nu=der)