import torch
from PyQt5.QtCore import QThread, pyqtSignal
import time
import numpy as np  
import control as ct

# Solo importamos la Red Neuronal
from Red_Hotend import HotendIdentifier, obtener_matrices_11_estados, WINDOW, HIDDEN

# ==============================================================
# FUNCIONES MATEMÁTICAS INYECTADAS DIRECTO EN EL WORKER
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
        N = np.linalg.pinv(M) @ rhs
    else:
        N = np.linalg.solve(M, rhs)

    Nx = N[0:n, :]
    Nu = N[n:n+1, :]
    Nbar = Nu + K @ Nx
    return Nbar.item()

def calcular_parametros_lqr_seguro(A, B, C, D, Ts=0.1):
    """
    Calcula el LQR con los 201 estados completos usando SciPy puro
    para evitar Slycot, y luego extrae solo los 6 valores más importantes
    para el Arduino.
    """
    Q = C.T @ C * 100
    R = np.array([[1]])

    try:
        # 1. Calculamos el LQR con la matriz gigante (SciPy lo hace sin Slycot)
        K, S, E = ct.dlqr(A, B, Q, R)
        
        # 2. Calculamos la prealimentación con el sistema completo
        Nbar = rscale_discrete(A, B, C, D, K)
        
        # 3. Extraemos SOLO las primeras 6 ganancias para el Arduino.
        # En la forma canónica, las primeras posiciones son los datos más recientes.
        K_lista = K.flatten()[:6].tolist()
        
        return K_lista, Nbar

    except Exception as e:
        print(f"⚠️ [Adaptativo] Error matemático crítico: {e}. Apagando heater por seguridad.")
        return [0.0] * 6, 0.0

# ==============================================================
# CLASE DEL HILO PRINCIPAL
# ==============================================================
class HiloMatematicoAdaptativo(QThread):
    nuevos_valores_ready = pyqtSignal(list, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.ventana_datos = [] 
        
        print("🧠 [Hilo Adaptativo] Iniciando... Cargando red neuronal.")
        try:
            input_dim = 2 * (WINDOW + 1)
            self.modelo = HotendIdentifier(input_dim=input_dim, hidden=HIDDEN)
            # Cargar los pesos entrenados usando map_location para forzar CPU
            self.modelo.load_state_dict(torch.load('hotend_identifier.pth', map_location=torch.device('cpu')))
            self.modelo.eval()
            print("✅ [Hilo Adaptativo] Red Neuronal lista para trabajar.")
        except Exception as e:
            print(f"❌ [Hilo Adaptativo] ERROR CRÍTICO al cargar el modelo .pth: {e}")

    def actualizar_ventana(self, ultimos_5_datos):
        self.ventana_datos = ultimos_5_datos

    def run(self):
        print("🚀 [Hilo Adaptativo] Motor matemático en marcha (Ciclo de 2s)...")
        
        while self.running:
            time.sleep(2) 

            if len(self.ventana_datos) < 5:
                continue 

            try:
                # Extraemos el monstruo de la red neuronal
                A_raw, B_raw, C_raw, D_raw = obtener_matrices_11_estados(self.modelo, self.ventana_datos)
                
                # Pasamos por nuestra función blindada que lo reduce a 6
                K_lista, Nbar = calcular_parametros_lqr_seguro(A_raw, B_raw, C_raw, D_raw, Ts=0.1)

                # DEBUG: Verificamos el tamaño exacto antes de mandar a la GUI
                print(f"🐞 DEBUG: Tamaño de K a enviar: {len(K_lista)}")
                if len(K_lista) != 6:
                    print("🚨 [ALERTA] ¡La lista no tiene 6 elementos! El Arduino va a fallar.")

                self.nuevos_valores_ready.emit(K_lista, Nbar)

            except Exception as e:
                print(f"⚠️ [Hilo Adaptativo] Fallo general en el ciclo: {e}")

    def stop(self):
        self.running = False
        self.wait()