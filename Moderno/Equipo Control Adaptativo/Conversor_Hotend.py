import numpy as np
import scipy.io
import control as ct
from pathlib import Path

# ==============================================================
# FUNCIÓN EQUIVALENTE A rscale_discrete DE MATLAB
# ==============================================================
def rscale_discrete(A, B, C, D, K):
    n = A.shape[0]
    M = np.block([
        [A - np.eye(n), B],
        [C, D]
    ])
    rhs = np.zeros((n + 1, 1))
    rhs[-1, 0] = 1.0

    if np.linalg.matrix_rank(M) < M.shape[0]:
        print("Advertencia: Matriz singular, usando pseudoinversa")
        N = np.linalg.pinv(M) @ rhs
    else:
        N = np.linalg.solve(M, rhs)

    Nx = N[0:n, :]
    Nu = N[n:n+1, :]
    Nbar = Nu + K @ Nx
    return Nbar.item()


# ==============================================================
# FUNCIÓN PARA EL HILO ADAPTATIVO (GUI)
# ==============================================================
def calcular_parametros_lqr(A, B, C, D, Ts=0.1):
    """
    Recibe las matrices crudas de la Red Neuronal, las reduce a 6,
    calcula el LQR y devuelve los números listos para el Arduino.
    """
    sys_full = ct.StateSpace(A, B, C, D, Ts)
    
    # Reducir el sistema (balred a 6 estados, sin la palabra tol)
    sys_r = ct.balred(sys_full, 6)
    Ar, Br, Cr, Dr = sys_r.A, sys_r.B, sys_r.C, sys_r.D
    
    # Diseñar el LQR sobre las matrices reducidas
    Q = Cr.T @ Cr * 100
    R = np.array([[1]])
    K, S, E = ct.dlqr(Ar, Br, Q, R)

    # Calcular prealimentación
    Nbar = rscale_discrete(Ar, Br, Cr, Dr, K)
    
    K_lista = K.flatten().tolist()
    return K_lista, Nbar


# ==============================================================
# MAIN - DISEÑO LQR HOTEND Y OBSERVADOR DE ESTADOS
# ==============================================================
if __name__ == '__main__':
    print("Calculando LQR y Observador del Hotend en Python...")

    BASE = Path(__file__).parent
    data = scipy.io.loadmat(BASE / 'state_space_hotend.mat')
    A = data['A']
    B = data['B']
    C = data['C']
    D = data['D']
    Ts = data['Ts'][0, 0]

    sys_full = ct.StateSpace(A, B, C, D, Ts)
    print(f"Sistema original: {sys_full.nstates} estados")

    # 1. Reducir el sistema (Modo Supervivencia sin Slycot)
    print("Cortando el sistema a 6 estados a pura fuerza bruta...")
    Ar = A[:6, :6]
    Br = B[:6, :]
    Cr = C[:, :6]
    Dr = D
    print(f"Sistema reducido a: {Ar.shape[0]} estados")

    # 2. Diseñar el LQR (Controlador de potencia)
    Q = Cr.T @ Cr * 100
    R = np.array([[1]])
    K, S, E = ct.dlqr(Ar, Br, Q, R)
    Nbar = rscale_discrete(Ar, Br, Cr, Dr, K)
    print(f"LQR calculado con éxito. Nbar: {Nbar:.6f}")

    # ==============================================================
    # 3. LA MAGIA PURISTA: Diseñar el Observador de Estados (Luenberger)
    # ==============================================================
    # Usamos las matemáticas de dlqr para calcular la ganancia L del observador
    Q_obs = np.eye(6) * 10  # Confiamos bastante en el modelo matemático
    R_obs = np.array([[1]]) # Peso del ruido del termistor
    
    L_obs, _, _ = ct.dlqr(Ar.T, Cr.T, Q_obs, R_obs)
    L = L_obs.T # Transponemos para que quede como vector vertical de 6x1

    # Imprimir para C++
    print("\n" + "="*50)
    print("✅ COPIA Y PEGA ESTOS ARREGLOS EN TU ARDUINO (.ino)")
    print("="*50)
    
    print(f"float C[6] = {{{', '.join([f'{val:.6f}' for val in Cr[0]])}}};")
    print(f"float L[6] = {{{', '.join([f'{val[0]:.6f}' for val in L])}}};")
    
    print("="*50 + "\n")

    # 4. Guardar el .mat para Simulink (opcional)
    scipy.io.savemat(BASE / 'lqr_valores_hotend.mat', {
        'K': K, 'Nbar': Nbar,
        'Ar': Ar, 'Br': Br, 'Cr': Cr, 'Dr': Dr
    })
    print("Archivo lqr_valores_hotend.mat guardado con éxito.")