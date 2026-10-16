import numpy as np
import config.parameters as _params


class SlidingPIController:
    """
    Controlador Sliding-PI en espacio articular (MATLAB/Simulink: system_762 'Controler' + system_3689 'PI').
    Ecuaciones del diagrama Simulink:
      delta_q  = q - q_d       (Sum6: '-+')
      delta_qp = qp - qp_d     (Sum7: '+-')
      S        = delta_qp + alpha * delta_q
      Ts       = -(Kp * S + Ki * integral(S dt))
    """
    def __init__(self, alpha=None, Kp=None, Ki=None, dt=None, kp=None, ki=None):
        self.alpha = float(_params.ALPHA_PI if alpha is None else alpha)
        self.Kp = float(_params.KP_S if (Kp is None and kp is None) else (Kp if Kp is not None else kp))
        self.Ki = float(_params.KI_S if (Ki is None and ki is None) else (Ki if Ki is not None else ki))
        self.dt = float(_params.DT if dt is None else dt)
        self.integral_S = np.zeros(6, dtype=np.float64)

    @property
    def kp(self):
        return self.Kp

    @kp.setter
    def kp(self, val):
        self.Kp = float(val)

    @property
    def ki(self):
        return self.Ki

    @ki.setter
    def ki(self, val):
        self.Ki = float(val)

    def reset(self):
        self.integral_S[:] = 0.0

    def step(self, delta_q, delta_qp, dt=None):
        """
        Paso de control directo a partir de errores delta_q = q - q_d y delta_qp = qp - qp_d.
        """
        dt_step = self.dt if dt is None else float(dt)
        dq = np.asarray(delta_q, dtype=np.float64)
        dqp = np.asarray(delta_qp, dtype=np.float64)
        S = dqp + self.alpha * dq
        self.integral_S += S * dt_step
        Ts = -(self.Kp * S + self.Ki * self.integral_S)
        return Ts

    def compute(self, q, qd, qp=None, qdp=None, dt=None):
        """
        Calcula Ts a partir de posiciones y velocidades medidas/deseadas (q, qd, qp, qdp),
        aplicando Sum6 (delta_q = q - qd) y Sum7 (delta_qp = qp - qdp) de Simulink.
        """
        dq = np.asarray(q, dtype=np.float64) - np.asarray(qd, dtype=np.float64)
        if qp is None or qdp is None:
            dqp = np.zeros_like(dq)
        else:
            dqp = np.asarray(qp, dtype=np.float64) - np.asarray(qdp, dtype=np.float64)
        return self.step(dq, dqp, dt=dt)