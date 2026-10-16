import numpy as np
import config.parameters as _params


def TIK(y, a0=None, D=None, rt=None, clamp_hemisphere: bool = True):
    """
    Task Inverse Kinematics (MATLAB: chart_45 'TIK' en GSPMe2026_03.slx).
    Aplica la restricción de la semiesfera superior:
      - Elevación b in [0, pi/2] (horizonte 0 rad a cenit pi/2 rad)
      - Origen de la plataforma sobre la semiesfera de radio rt centrada en D:
        da = D + rt * R[:, 2], con ||da - D|| = rt y da_z = D_z + rt * sin(b) >= D_z.
    """
    if a0 is None:
        a0 = _params.ALPHA_0
    if D is None:
        D = _params.D
    if rt is None:
        rt = _params.RT

    a = float(y[0])
    b = float(y[1])
    if clamp_hemisphere:
        b = float(np.clip(b, 0.0, np.pi / 2.0))

    D_vec = np.asarray(D, dtype=np.float64).reshape(3)
    rt_val = float(rt)

    R = np.array([
        # Fila 1
        [np.sin(a) * np.sin(a - a0) + np.cos(a) * np.cos(a - a0) * np.sin(b),
         np.cos(a - a0) * np.sin(a) - np.cos(a) * np.sin(a - a0) * np.sin(b),
         np.cos(a) * np.cos(b)],

        # Fila 2
        [np.cos(a) * np.sin(a - a0) - np.cos(a - a0) * np.sin(a) * np.sin(b),
         np.cos(a - a0) * np.cos(a) + np.sin(a) * np.sin(a - a0) * np.sin(b),
         -np.cos(b) * np.sin(a)],

        # Fila 3
        [-np.cos(a - a0) * np.cos(b),
          np.cos(b) * np.sin(a - a0),
          np.sin(b)]
    ], dtype=np.float64)

    da = D_vec + rt_val * R[:, 2]
    return da, R


def PIK(da, R, Az=None, Bz=None):
    """
    Platform Inverse Kinematics (Gough-Stewart, MATLAB: chart_56 'PIK').
    -------------------------------------------------------------------
    Az : puntos de anclaje en la plataforma móvil (6x3) o (3x6)
    Bz : puntos de anclaje en la base fija     (6x3) o (3x6)
    da : posición deseada del origen de la plataforma (3,)
    R  : matriz de rotación deseada 3x3

    Returns
    -------
    q : longitudes totales ancla-a-ancla de las 6 piernas (6,) en mm
    """
    if Az is None:
        Az = _params.Az
    if Bz is None:
        Bz = _params.Bz

    Az_arr = np.asarray(Az, dtype=np.float64)
    Bz_arr = np.asarray(Bz, dtype=np.float64)
    if Az_arr.shape == (3, 6):
        Az_arr = Az_arr.T
    if Bz_arr.shape == (3, 6):
        Bz_arr = Bz_arr.T

    da_vec = np.asarray(da, dtype=np.float64).reshape(3)
    R_mat = np.asarray(R, dtype=np.float64).reshape(3, 3)

    q = np.zeros(6, dtype=np.float64)
    for i in range(6):
        leg = da_vec + R_mat @ Az_arr[i] - Bz_arr[i]
        q[i] = np.linalg.norm(leg)
    return q


def home_kinematics(D=None, Az=None, Bz=None):
    """
    Cinemática en la posición HOME de reposo (todos los actuadores retraídos: 0 mm de extensión).
    En HOME:
      - da = D (sin desplazamiento de semiesfera rt)
      - R = I_3 (plataforma horizontal apuntando al cenit: alpha=0, beta=90°)
      - q_mm = PIK(D, I_3) = [L0, L0, L0, L0, L0, L0] (longitud retraída)
      - q_ef = q_mm - L0 = [0, 0, 0, 0, 0, 0] mm (0% de carrera)
    """
    if D is None:
        D = _params.D
    da_home = np.asarray(D, dtype=np.float64).reshape(3).copy()
    R_home = np.eye(3, dtype=np.float64)
    q_home = PIK(da_home, R_home, Az=Az, Bz=Bz)
    return q_home, da_home, R_home


def inverse_kinematics(y, rt=None, a0=None, D=None, Az=None, Bz=None, clamp_hemisphere: bool = True):
    """Función principal: y=[α, β elevación] en rad → q_mm (largo ancla a ancla, sin restar L0).

    rt debe ser el de los parámetros cargados (parameters.json); da = D + rt·R[:,2].
    Para obtener la extensión efectiva del actuador (0 mm en HOME): q_ef = q_mm - L0.
    """
    da, R = TIK(y, a0=a0, D=D, rt=rt, clamp_hemisphere=clamp_hemisphere)
    q_mm = PIK(da, R, Az=Az, Bz=Bz)
    return q_mm, da, R