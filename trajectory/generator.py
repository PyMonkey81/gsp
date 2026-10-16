import numpy as np
from logic.spline import SimpleCubicSpline as CubicSpline
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class TrajectoryConfig:
    t_home_end: float = 60.0
    t_tracking_end: float = 653.0
    dt: float = 0.001
    filter_wn: float = 3.4


class SecondOrderFilter:
    """Filtro de 2º orden (equivalente a Filter2 de Simulink)"""
    def __init__(self, wn: float = 3.4, dt: float = 0.001):
        self.wn = wn
        self.dt = dt
        self.x1 = 0.0
        self.x2 = 0.0

    def reset(self, x1: float = 0.0, x2: float = 0.0):
        self.x1 = x1
        self.x2 = x2

    def step(self, ur: float, dt: float = None) -> Tuple[float, float]:
        h = self.dt if dt is None else dt
        error1 = ur - self.x1
        error2 = self.wn * error1 - 2.0 * self.x2
        x2_dot = self.wn * error2
        x1_dot = self.x2

        self.x2 += x2_dot * h
        self.x1 += x1_dot * h
        return self.x1, self.x2


class TrajectoryGenerator:
    """
    Generador de trayectoria para la orientación (α, β).
    
    Fases:
      1. Home      → lleva suavemente al primer punto de tracking
      2. Tracking  → sigue el spline definido por los 3 puntos
      3. Return    → regresa suavemente a (0, 0)
    """

    def __init__(self, config: TrajectoryConfig = TrajectoryConfig()):
        self.cfg = config

        # Un filtro por cada ángulo
        self.filter_a = SecondOrderFilter(wn=config.filter_wn, dt=config.dt)
        self.filter_b = SecondOrderFilter(wn=config.filter_wn, dt=config.dt)

        self.spline_a: Optional[CubicSpline] = None
        self.spline_b: Optional[CubicSpline] = None

        self.y_start_a = 0.0
        self.y_start_b = 0.0

        self.reset()

    def set_tracking_points(self, points: list):
        """
        points: lista de 3 pares [[α1, β1], [α2, β2], [α3, β3]] en radianes
        
        β es la elevación desde horizonte (LMT: 10.10, 45.60, 10.00), NO es tilt.
        Se construye el spline directamente con los valores sin conversión.
        
        Se construye un spline en el intervalo [t_home_end, t_tracking_end]
        """
        points = np.asarray(points, dtype=float)
        if points.shape != (3, 2):
            raise ValueError("Se esperaban exactamente 3 puntos (α, β)")

        t0 = self.cfg.t_home_end
        t1 = self.cfg.t_tracking_end

        # Tiempos de los 3 puntos (inicio, medio, final de la fase de tracking)
        times = np.array([
            t0,
            (t0 + t1) / 2.0,
            t1
        ])

        alpha = points[:, 0]  # already in radians
        beta_rad = points[:, 1]  # already in radians (elevation from horizon, NOT tilt)

        self.spline_a = CubicSpline(times, alpha, bc_type='natural')
        self.spline_b = CubicSpline(times, beta_rad,  bc_type='natural')

        # Punto al que debe llegar la fase Home
        self.y_start_a = float(self.spline_a(t0))
        self.y_start_b = float(self.spline_b(t0))

    def step(self, t: float, dt: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Genera la referencia en el instante t.

        Fases:
          0..t_home: sube desde home hasta el primer punto del tracking
          t_home..t_tracking_end: spline completo
          t_tracking_end..t_tracking_end + t_home: retorno a home

        Returns
        -------
        y  : np.array([α, β])
        yp : np.array([α_dot, β_dot])
        """
        t_home = self.cfg.t_home_end
        t_end = self.cfg.t_tracking_end
        t_return_end = t_end + t_home

        # -------------------------------------------------
        # FASE 1: Home → primer punto de tracking
        # -------------------------------------------------
        if t < t_home:
            progress = t / t_home if t_home > 0 else 1.0
            # TIK: beta es elevación desde el horizonte. Cenit (HOME) = pi/2, no 0.
            # La rampa sale de (0, pi/2) hacia el punto 1 de la tabla.
            ur_a = self.y_start_a * progress
            ur_b = (np.pi / 2) + (self.y_start_b - np.pi / 2) * progress

            ya, ypa = self.filter_a.step(ur_a, dt)
            yb, ypb = self.filter_b.step(ur_b, dt)

            self.phase = "home"

        # -------------------------------------------------
        # FASE 2: Tracking (spline)
        # -------------------------------------------------
        elif t < t_end:
            if self.phase != "tracking":
                self.filter_a.reset(x1=self.y_start_a, x2=0.0)
                self.filter_b.reset(x1=self.y_start_b, x2=0.0)
                self.phase = "tracking"

            if self.spline_a is None or self.spline_b is None:
                ya, ypa = 0.0, 0.0
                yb, ypb = 0.0, 0.0
            else:
                ya = float(self.spline_a(t))
                ypa = float(self.spline_a(t, 1))
                yb = float(self.spline_b(t))
                ypb = float(self.spline_b(t, 1))

        # -------------------------------------------------
        # FASE 3: Retorno a home
        # -------------------------------------------------
        elif t < t_return_end:
            if self.phase != "return":
                self.filter_a.reset(x1=self.y_current[0], x2=self.yp_current[0])
                self.filter_b.reset(x1=self.y_current[1], x2=self.yp_current[1])
                self.y_return_start = self.y_current.copy()
                self.phase = "return"

            return_duration = max(t_return_end - t_end, 1e-6)
            local_t = max(t - t_end, 0.0)
            progress = min(local_t / return_duration, 1.0)

            # Retorno al cenit (alpha 0, beta = pi/2), no a beta 0 (horizonte).
            target_a = self.y_return_start[0] * (1.0 - progress)
            target_b = self.y_return_start[1] + (np.pi / 2 - self.y_return_start[1]) * progress

            ya, ypa = self.filter_a.step(target_a, dt)
            yb, ypb = self.filter_b.step(target_b, dt)

        else:
            ya, ypa = 0.0, 0.0
            yb, ypb = np.pi / 2, 0.0
            self.phase = "done"

        self.y_current = np.array([ya, yb])
        self.yp_current = np.array([ypa, ypb])

        return self.y_current.copy(), self.yp_current.copy()

    def reset(self):
        """Reset estado interno: filtros en cenit, no en el horizonte."""
        self.filter_a.reset(0.0, 0.0)
        self.filter_b.reset(np.pi / 2, 0.0)
        self.phase = "home"
        self.y_current = np.array([0.0, np.pi / 2])
        self.yp_current = np.array([0.0, 0.0])

    def generate(self, t_final: float = 800.0):
        """Genera toda la trayectoria offline (útil para plots)."""
        t = np.arange(0.0, t_final, self.cfg.dt)
        yd  = np.zeros((len(t), 2))
        ypd = np.zeros((len(t), 2))

        self.filter_a.reset()
        self.filter_b.reset()
        self.phase = "home"

        for i, ti in enumerate(t):
            yd[i], ypd[i] = self.step(ti)

        return t, yd, ypd