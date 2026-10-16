"""
Direct Kinematics module for the 6-DOF Gough-Stewart Platform.
Based on the Simulink model GSPMe2026_03.slx:
- CA(R): 'Task Space Direct Kinematics / Angulos de Sensor' (chart_76.xml, SID=1588)
- TSDK(R, Omega): 'Task Space Direct Kinematics' subsystem (system_1585.xml, SID=1585)
- TSVK(y, yp, rt): 'Task Space Velocity Kinematics' subsystem (system_722.xml, SID=722)
- PDK(q, ...): 6-DOF Platform Direct Kinematics (Newton-Raphson with InvJac & PIK)
- direct_kinematics(q, ...): Complete Direct Kinematics pipeline (q, qp -> Y, Yp, da, R, Va, Omega)
"""

import numpy as np
from config.parameters import Az, Bz, ALPHA_0, D, RT
from kinematics.inverse import TIK, PIK
from kinematics.jacobian import InvJac


def CA(R: np.ndarray):
    """
    Cálculo de Alpha y Betha reales a partir de la matriz de rotación R(t).
    Equivalente exacto al bloque MATLAB Function 'Angulos de Sensor'
    en 'Task Space Direct Kinematics' (chart_76.xml, SID=1588):

        function [alpha,betha] = CA(R)
        %Calculo de Alpha y betha reales
        alpha=((-atan2(R(2,3),R(1,3))));
        betha=((atan(R(3,3)/(sqrt((R(1,3))^2+(R(2,3))^2)))));

    Parameters
    ----------
    R : (3, 3) np.ndarray
        Matriz de rotación actual de la plataforma móvil R(t).

    Returns
    -------
    alpha : float
        Ángulo de azimut real en radianes.
    betha : float
        Ángulo de elevación real desde el horizonte en radianes.
    """
    R = np.asarray(R, dtype=float)
    r13 = float(R[0, 2])
    r23 = float(R[1, 2])
    r33 = float(R[2, 2])

    alpha = float(-np.arctan2(r23, r13))
    denom = float(np.sqrt(r13 * r13 + r23 * r23))
    # atan2(r33, denom) es idéntico a atan(r33 / denom) para denom > 0 y evita división por cero en cenit
    betha = float(np.arctan2(r33, denom))
    return alpha, betha


def TSDK(R: np.ndarray, Omega: np.ndarray = None):
    """
    Task Space Direct Kinematics (Simulink: system_1585.xml, SID=1585).
    Calcula el vector de orientación en el espacio de tarea Y = [alpha, betha]
    y su derivada temporal Yp = [alpha_dot, betha_dot] a partir de R(t) y Omega(t).

    Estructura de bloques en system_1585.xml:
      1. [alpha, betha] = CA(R) (SID=1588) -> Mux (SID=1599) -> Outport Y (SID=1604)
      2. Matriz 2x3 (SID=1595..1598):
         [ cos(alpha)*tan(betha), -sin(alpha)*tan(betha), -1 ]
         [           -sin(alpha),            -cos(alpha),  0 ]
      3. Yp = M(alpha, betha) @ Omega (SID=1592) -> Outport Yp (SID=1605)

    Parameters
    ----------
    R : (3, 3) np.ndarray
        Matriz de rotación R(t) de la plataforma móvil.
    Omega : (3,) np.ndarray, optional
        Velocidad angular [wx, wy, wz] en el marco del mundo (rad/s).
        Si es None, retorna Yp = [0.0, 0.0].

    Returns
    -------
    Y : (2,) np.ndarray
        Vector [alpha, betha] en radianes.
    Yp : (2,) np.ndarray
        Vector [alpha_dot, betha_dot] en rad/s.
    """
    alpha, betha = CA(R)
    Y = np.array([alpha, betha], dtype=float)

    if Omega is None:
        return Y, np.zeros(2, dtype=float)

    Omega = np.asarray(Omega, dtype=float).reshape(3)
    sa, ca = np.sin(alpha), np.cos(alpha)
    sb, cb = np.sin(betha), np.cos(betha)

    # Divide (SID=1589): sin(betha) / cos(betha) = tan(betha)
    tan_b = sb / cb if abs(cb) > 1e-12 else 0.0

    # Matrix Concatenate (SID=1595, 2x3)
    M = np.array([
        [ca * tan_b, -sa * tan_b, -1.0],
        [-sa,        -ca,          0.0]
    ], dtype=float)

    Yp = M @ Omega
    return Y, Yp


def TSVK(y: np.ndarray, yp: np.ndarray = None, rt: float = RT) -> np.ndarray:
    """
    Task Space Velocity Kinematics (Simulink: system_722.xml, SID=722).
    Mapea la posición y = [alpha, betha] y velocidad yp = [alpha_dot, betha_dot]
    del espacio de tarea al twist de la plataforma twist_a = [Va; Wa] (6x1).

    Si `yp` es None, devuelve la matriz Jacobiana de tarea J_ts (6x2) tal que
    twist_a = J_ts @ yp.

    Parameters
    ----------
    y : (2,) array-like
        Ángulos [alpha, betha] en radianes.
    yp : (2,) array-like, optional
        Velocidades angulares [alpha_dot, betha_dot] en rad/s.
    rt : float
        Radio de la esfera virtual (mm o m según parámetros).

    Returns
    -------
    twist_a : (6,) np.ndarray o J_ts : (6, 2) np.ndarray
        Vector [vx, vy, vz, wx, wy, wz] si se proporciona `yp`, o matriz (6, 2) si `yp` es None.
    """
    a, b = float(y[0]), float(y[1])
    sa, ca = np.sin(a), np.cos(a)
    sb, cb = np.sin(b), np.cos(b)

    # Matriz 6x2 exacta de system_722.xml (Matrix Concatenate3 SID=745 y Matrix Concatenate1 SID=744)
    J_ts = np.array([
        [-rt * sa * cb, -rt * ca * sb],
        [-rt * ca * cb,  rt * sa * sb],
        [0.0,            rt * cb],
        [ca * cb,       -sa],
        [-sa * cb,      -ca],
        [sb - 1.0,       0.0]
    ], dtype=float)

    if yp is None:
        return J_ts
    yp_arr = np.asarray(yp, dtype=float).reshape(2)
    return J_ts @ yp_arr


def R_to_rpy(R: np.ndarray) -> np.ndarray:
    """
    Extrae los ángulos de Euler (roll, pitch, yaw) en radianes a partir de R,
    inversa de `rpy_to_R(roll, pitch, yaw)` en kinematics/pose.py (convención ZYX).
    """
    R = np.asarray(R, dtype=float)
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    cp = np.cos(pitch)
    if abs(cp) > 1e-8:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll = 0.0
        yaw = float(np.arctan2(-R[0, 1], R[1, 1]))
    return np.array([roll, pitch, yaw], dtype=float)


def _rodrigues(w: np.ndarray) -> np.ndarray:
    """Mapa exponencial SO(3): vector de rotación w (3,) -> matriz de rotación 3x3."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3) + np.array([
            [0.0, -w[2], w[1]],
            [w[2], 0.0, -w[0]],
            [-w[1], w[0], 0.0]
        ], dtype=float)
    k = w / th
    K = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0]
    ], dtype=float)
    return np.eye(3) + np.sin(th) * K + (1.0 - np.cos(th)) * (K @ K)


def _solve_task_space_2dof(
    q: np.ndarray,
    y0: np.ndarray = None,
    Az: np.ndarray = Az,
    Bz: np.ndarray = Bz,
    D: np.ndarray = D,
    rt: float = RT,
    a0: float = ALPHA_0,
    tol: float = 1e-9,
    max_iter: int = 40
):
    """
    Resuelve la cinemática directa sobre la variedad de tarea 2-DOF (alpha, betha)
    usando Gauss-Newton con el Jacobiano compuesto J_q_y = InvJac(da, R) @ TSVK(y, rt).
    """
    if y0 is None:
        # Búsqueda gruesa rápida sobre el hemisferio físico para elegir la mejor semilla
        best_y = np.array([0.0, np.pi / 2], dtype=float)
        da_c, R_c = TIK(best_y, a0=a0, D=D, rt=rt)
        best_err = float(np.linalg.norm(q - PIK(da_c, R_c, Az=Az, Bz=Bz)))
        if best_err > 1e-3:
            for b_deg in (70.0, 45.0, 20.0):
                b_rad = np.deg2rad(b_deg)
                for a_deg in np.linspace(-180.0, 150.0, 12):
                    y_try = np.array([np.deg2rad(a_deg), b_rad], dtype=float)
                    da_t, R_t = TIK(y_try, a0=a0, D=D, rt=rt)
                    err_t = float(np.linalg.norm(q - PIK(da_t, R_t, Az=Az, Bz=Bz)))
                    if err_t < best_err:
                        best_err = err_t
                        best_y = y_try
        y = best_y
    else:
        y = np.asarray(y0, dtype=float).copy()

    da, R = TIK(y, a0=a0, D=D, rt=rt)
    for _ in range(max_iter):
        q_est = PIK(da, R, Az=Az, Bz=Bz)
        dq = q - q_est
        err = float(np.linalg.norm(dq))
        if err < tol:
            return y, da, R, err

        A, _ = InvJac(da, R, Az=Az, Bz=Bz)
        J_ts = TSVK(y, yp=None, rt=rt)
        J_qy = A @ J_ts  # (6, 2)

        dy, *_ = np.linalg.lstsq(J_qy, dq, rcond=None)
        # Búsqueda de línea amortiguada
        step = 1.0
        for _ls in range(8):
            y_new = np.array([
                np.arctan2(np.sin(y[0] + step * dy[0]), np.cos(y[0] + step * dy[0])),
                np.clip(y[1] + step * dy[1], -np.pi / 2, np.pi / 2)
            ], dtype=float)
            da_new, R_new = TIK(y_new, a0=a0, D=D, rt=rt)
            if np.linalg.norm(q - PIK(da_new, R_new, Az=Az, Bz=Bz)) < err:
                y, da, R = y_new, da_new, R_new
                break
            step *= 0.5
        else:
            y = y + 0.1 * dy
            da, R = TIK(y, a0=a0, D=D, rt=rt)

    err = float(np.linalg.norm(q - PIK(da, R, Az=Az, Bz=Bz)))
    return y, da, R, err


def PDK(
    q: np.ndarray,
    da0: np.ndarray = None,
    R0: np.ndarray = None,
    Az: np.ndarray = Az,
    Bz: np.ndarray = Bz,
    D: np.ndarray = D,
    rt: float = RT,
    a0: float = ALPHA_0,
    tol: float = 1e-9,
    max_iter: int = 60
):
    """
    Platform Direct Kinematics (6-DOF Gough-Stewart).
    Calcula la posición `da` (3,) y matriz de rotación `R` (3, 3) de la plataforma
    móvil a partir de las 6 longitudes de los actuadores `q` (ancla a ancla)
    mediante el método de Newton-Raphson usando `PIK` e `InvJac`.

    Parameters
    ----------
    q : (6,) array-like
        Longitudes de las 6 piernas (ancla a ancla).
    da0 : (3,) array-like, optional
        Estimación inicial de posición `da`.
    R0 : (3, 3) array-like, optional
        Estimación inicial de rotación `R`.
    Az, Bz : (6, 3) np.ndarray
        Puntos de anclaje en plataforma móvil y base fija.
    D : (3,) np.ndarray
        Vector de posición nominal de referencia.
    rt : float
        Radio de la esfera virtual.
    a0 : float
        Ángulo acimutal de referencia Alpha_0.
    tol : float
        Tolerancia de convergencia en la norma del residuo de longitudes.
    max_iter : int
        Número máximo de iteraciones de Newton-Raphson.

    Returns
    -------
    da : (3,) np.ndarray
        Posición estimada del origen de la plataforma móvil.
    R : (3, 3) np.ndarray
        Matriz de rotación estimada de la plataforma móvil.
    """
    q = np.asarray(q, dtype=float).reshape(6)

    if da0 is None or R0 is None:
        # Primero intentamos resolver en la variedad de tarea 2-DOF (TIK) para obtener
        # la semilla física óptima en el hemisferio superior (o la solución exacta si pertenece a TIK).
        _, da_seed, R_seed, err_2d = _solve_task_space_2dof(
            q, Az=Az, Bz=Bz, D=D, rt=rt, a0=a0, tol=tol
        )
        if err_2d < tol:
            return da_seed, R_seed
        # Si q proviene de una pose cartesiana general (fuera de la esfera TIK),
        # comparamos la semilla de tarea contra la semilla home (D, I).
        err_home = float(np.linalg.norm(q - PIK(D, np.eye(3), Az=Az, Bz=Bz)))
        if err_home < err_2d:
            da = np.asarray(D, dtype=float).copy()
            R = np.eye(3, dtype=float)
        else:
            da = da_seed.copy()
            R = R_seed.copy()
    else:
        da = np.asarray(da0, dtype=float).copy()
        R = np.asarray(R0, dtype=float).copy()

    for _ in range(max_iter):
        q_est = PIK(da, R, Az=Az, Bz=Bz)
        dq = q - q_est
        err = float(np.linalg.norm(dq))
        if err < tol:
            break

        A, _ = InvJac(da, R, Az=Az, Bz=Bz)
        dx, *_ = np.linalg.lstsq(A, dq, rcond=None)

        # Limitar paso angular máximo por iteración para evitar cruzar singularidades
        w_norm = float(np.linalg.norm(dx[3:]))
        step_scale = min(1.0, 0.2 / max(w_norm, 1e-12))

        for _ls in range(8):
            da_new = da + step_scale * dx[:3]
            R_new = _rodrigues(step_scale * dx[3:]) @ R
            if np.linalg.norm(q - PIK(da_new, R_new, Az=Az, Bz=Bz)) < err:
                da, R = da_new, R_new
                break
            step_scale *= 0.5
        else:
            da = da + 0.05 * dx[:3]
            R = _rodrigues(0.05 * dx[3:]) @ R

    return da, R


def direct_kinematics(
    q: np.ndarray,
    qp: np.ndarray = None,
    da0: np.ndarray = None,
    R0: np.ndarray = None,
    Az: np.ndarray = Az,
    Bz: np.ndarray = Bz,
    D: np.ndarray = D,
    rt: float = RT,
    a0: float = ALPHA_0,
    tol: float = 1e-9,
    max_iter: int = 60
):
    """
    Función principal de Cinemática Directa completa (Plataforma + Espacio de Tarea).
    Conecta la resolución geométrica de la plataforma (PDK + InvJac) con el bloque
    'Task Space Direct Kinematics' (TSDK / CA) de Simulink (GSPMe2026_03.slx):

      1. q -> (da, R) mediante PDK(q)
      2. Si se proporciona `qp` (velocidades de actuadores, 6x1):
         [Va; Omega] = A(da, R)^{-1} @ qp
      3. (R, Omega) -> (Y, Yp) mediante TSDK(R, Omega), donde:
         Y  = [alpha, betha] (rad)
         Yp = [alpha_dot, betha_dot] (rad/s)

    Parameters
    ----------
    q : (6,) array-like
        Longitudes de los 6 actuadores (ancla a ancla, mm).
    qp : (6,) array-like, optional
        Velocidades de extensión de los 6 actuadores (mm/s).
    da0, R0 : optional
        Semillas iniciales para posición (3,) y rotación (3, 3).
    Az, Bz, D, rt, a0 : optional
        Parámetros geométricos de la plataforma.

    Returns
    -------
     Si `qp` es None:
        Y : (2,) np.ndarray -> [alpha, betha] en radianes
        da : (3,) np.ndarray -> posición del origen de la plataforma (mm)
        R : (3, 3) np.ndarray -> matriz de rotación 3x3
    Si `qp` se proporciona:
        Y : (2,) np.ndarray -> [alpha, betha] en radianes
        Yp : (2,) np.ndarray -> [alpha_dot, betha_dot] en rad/s
        da : (3,) np.ndarray -> posición del origen de la plataforma (mm)
        R : (3, 3) np.ndarray -> matriz de rotación 3x3
        Va : (3,) np.ndarray -> velocidad lineal de la plataforma (mm/s)
        Omega : (3,) np.ndarray -> velocidad angular de la plataforma (rad/s)
    """
    da, R = PDK(
        q, da0=da0, R0=R0, Az=Az, Bz=Bz, D=D, rt=rt, a0=a0, tol=tol, max_iter=max_iter
    )

    if qp is None:
        Y, _ = TSDK(R, Omega=None)
        return Y, da, R

    qp_arr = np.asarray(qp, dtype=float).reshape(6)
    A, _ = InvJac(da, R, Az=Az, Bz=Bz)
    twist, *_ = np.linalg.lstsq(A, qp_arr, rcond=None)
    Va = twist[:3]
    Omega = twist[3:]

    Y, Yp = TSDK(R, Omega=Omega)
    return Y, Yp, da, R, Va, Omega
