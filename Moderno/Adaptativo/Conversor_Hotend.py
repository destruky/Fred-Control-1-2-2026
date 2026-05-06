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
    Recibe las matrices crudas de la Red Neuronal, las reduce,
    calcula el LQR y devuelve los números listos para el Arduino.
    """
    # 1. Crear el sistema completo
    sys_full = ct.StateSpace(A, B, C, D, Ts)
    
    # 2. Reducir el sistema (minreal)
    sys_r = ct.minreal(sys_full, tol=1e-6)
    Ar, Br, Cr, Dr = sys_r.A, sys_r.B, sys_r.C, sys_r.D
    
    # 3. Diseñar el LQR sobre las matrices reducidas
    Q = Cr.T @ Cr * 100
    R = np.array([[1]])
    K, S, E = ct.dlqr(Ar, Br, Q, R)

    # 4. Calcular prealimentación
    Nbar = rscale_discrete(Ar, Br, Cr, Dr, K)
    
    # 5. Convertir K (matriz de numpy) a una simple lista de Python
    # Esto es vital para poder mandarla por texto en el puerto Serial
    K_lista = K.flatten().tolist()
    
    return K_lista, Nbar


# ==============================================================
# MAIN - DISEÑO LQR HOTEND EN PYTHON
# ==============================================================
if __name__ == '__main__':
    print("Calculando LQR del Hotend en Python...")

    # Obtener la ruta exacta donde vive este script
    BASE = Path(__file__).parent

    # 1. Cargar las matrices usando la ruta BASE
    # Asegúrate de que state_space_hotend.mat exista en la misma carpeta
    data = scipy.io.loadmat(BASE / 'state_space_hotend.mat')
    A = data['A']
    B = data['B']
    C = data['C']
    D = data['D']
    Ts = data['Ts'][0, 0]

    # Crear el sistema completo en la librería control
    sys_full = ct.StateSpace(A, B, C, D, Ts)
    print(f"Sistema original: {sys_full.nstates} estados")

    # 2. Reducir el sistema (Equivalente al minreal de MATLAB)
    print("Reduciendo sistema (esto puede tardar un par de segundos)...")
    sys_r = ct.minreal(sys_full, tol=1e-6)
    Ar, Br, Cr, Dr = sys_r.A, sys_r.B, sys_r.C, sys_r.D
    print(f"Sistema reducido a: {sys_r.nstates} estados")

    # 3. Diseñar el LQR sobre las matrices reducidas
    Q = Cr.T @ Cr * 100
    R = np.array([[1]])
    
    K, S, E = ct.dlqr(Ar, Br, Q, R)

    # 4. Calcular prealimentación
    Nbar = rscale_discrete(Ar, Br, Cr, Dr, K)
    
    print("LQR calculado con éxito.")
    print(f"Nbar calculado: {Nbar:.6f}")

    # 5. Guardar el .mat para Simulink en la misma ruta
    scipy.io.savemat(BASE / 'lqr_valores_hotend.mat', {
        'K': K, 
        'Nbar': Nbar,
        'Ar': Ar, 
        'Br': Br, 
        'Cr': Cr, 
        'Dr': Dr
    })
    print("\n✅ Archivo lqr_valores_hotend.mat guardado con éxito.")