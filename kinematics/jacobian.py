import numpy as np
from config.parameters import Az, Bz

def InvJac(da, R, Az=Az, Bz=Bz):
    A = np.zeros((6, 6))
    L1 = np.zeros((3, 6))

    for i in range(6):
        leg = da + R @ Az[i] - Bz[i]
        s_i = leg / np.linalg.norm(leg)
        L1[:, i] = s_i
        Ra_i = R @ Az[i]
        A[i, :3] = s_i
        A[i, 3:] = np.cross(Ra_i, s_i)

    return A, L1



# def InvJac(Az: np.ndarray, Bz: np.ndarray, da: np.ndarray, R: np.ndarray):
#     """
#     Jacobiano Inverso de la plataforma Gough-Stewart.
    
#     Parameters
#     ----------
#     Az : (6,3) puntos de anclaje de la plataforma móvil
#     Bz : (6,3) puntos de anclaje de la base
#     da : (3,)  posición del origen de la plataforma
#     R  : (3,3) matriz de rotación
    
#     Returns
#     -------
#     A  : (6,6) matriz Jacobiana inversa
#     L1 : (3,6) vectores unitarios de las piernas
#     """
#     if Az.shape == (3, 6):
#         Az = Az.T
#     if Bz.shape == (3, 6):
#         Bz = Bz.T

#     A = np.zeros((6, 6))
#     L1 = np.zeros((3, 6))

#     for i in range(6):
#         # Vector de la pierna
#         leg = da + R @ Az[i, :] - Bz[i, :]
#         q_i = np.linalg.norm(leg)
        
#         # Vector unitario
#         s_i = leg / q_i
#         L1[:, i] = s_i
        
#         # Fila del Jacobiano: [s_i^T , (R*a_i × s_i)^T]
#         Ra_i = R @ Az[i, :]
#         A[i, :3] = s_i
#         A[i, 3:] = np.cross(Ra_i, s_i)

#     return A, L1