import numpy as np
from config.parameters import ALPHA_PI, KP_S, KI_S, DT

class SlidingPIController:
    def __init__(self, alpha=ALPHA_PI, Kp=KP_S, Ki=KI_S, dt=DT):
        self.alpha = alpha
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt
        self.integral_S = np.zeros(6)

    def reset(self):
        self.integral_S[:] = 0.0

    def step(self, delta_q, delta_qp):
        S = delta_qp + self.alpha * delta_q
        self.integral_S += S * self.dt
        Ts = -(self.Kp * S + self.Ki * self.integral_S)
        return Ts