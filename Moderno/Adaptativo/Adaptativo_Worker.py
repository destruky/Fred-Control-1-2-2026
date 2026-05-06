from PyQt5.QtCore import QThread, pyqtSignal
import torch
import time
from pathlib import Path

# ==============================================================
# IMPORTANDO TUS LIBRERÍAS LIMPIAS
# ==============================================================
from Red_Hotend import HotendNARX, HotendIdentifier, obtener_matrices_11_estados, WINDOW, HIDDEN
from Conversor_Hotend import calcular_parametros_lqr

_BASE = Path(__file__).parent
_PTH  = _BASE.parent / "NN" / "Hotend" / "hotend_identifier.pth"

class HiloMatematicoAdaptativo(QThread):
    # Esta señal enviará la lista de K (6 valores) y el escalar Nbar a la GUI
    nuevos_valores_ready = pyqtSignal(list, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.ventana_datos = [] # Últimos WINDOW=100 datos [[temp,pwm], ...]
        
        print("🧠 [Hilo Adaptativo] Iniciando... Cargando red neuronal en memoria RAM.")
        try:
            # Arquitectura real: W=100, input_dim=2*(W+1)=202, hidden=128
            input_dim = 2 * (WINDOW + 1)
            self.modelo = HotendIdentifier(input_dim=input_dim, hidden=HIDDEN)
            self.modelo.load_state_dict(torch.load(str(_PTH), map_location=torch.device('cpu')))
            self.modelo.eval() # Modo inferencia
            print("✅ [Hilo Adaptativo] Red Neuronal lista para trabajar.")
        except Exception as e:
            print(f"❌ [Hilo Adaptativo] ERROR CRÍTICO al cargar el modelo .pth: {e}")

    def actualizar_ventana(self, ultimos_5_datos):
        """
        La GUI llamará a esta función cada 100ms para inyectar datos frescos.
        ultimos_5_datos debe ser una lista de 5 listas: [[temp1, pwm1], ..., [temp5, pwm5]]
        """
        self.ventana_datos = ultimos_5_datos

    def run(self):
        print("🚀 [Hilo Adaptativo] Motor matemático en marcha (Ciclo de 2s)...")
        
        while self.running:
            time.sleep(2) # Separación de Escalas de Tiempo (Lazo lento)

            # Si la GUI aún no ha juntado 5 datos, nos esperamos
            if len(self.ventana_datos) < WINDOW:
                continue 

            try:
                # ==========================================
                # ZONA DE INFERENCIA Y CÁLCULO
                # ==========================================
                
                # PASO A: Extraer matrices de 11 estados usando el Jacobiano
                A11, B11, C11, D11 = obtener_matrices_11_estados(self.modelo, self.ventana_datos)
                
                # PASO B: Reducir a 6 estados y resolver ecuación de Riccati (dlqr)
                # Ts = 0.1 se asume por defecto en la función que creaste
                K_lista, Nbar = calcular_parametros_lqr(A11, B11, C11, D11, Ts=0.1)

                # PASO C: ¡Éxito! Avisamos a la GUI que hay números nuevos
                self.nuevos_valores_ready.emit(K_lista, Nbar)
                # print(f"🔄 LQR Actualizado -> Nbar: {Nbar:.3f}") # Descomentar para debug en consola

            except Exception as e:
                # EL ESCUDO: Si la matriz es singular o los datos son ruido puro,
                # ignoramos este ciclo y el Arduino seguirá con su matriz anterior.
                print(f"⚠️ [Hilo Adaptativo] Ruido detectado. Falló la matemática: {e}")
                print("🛡️ Abortando actualización. Manteniendo el control actual seguro.")

    def stop(self):
        """Detiene el hilo de forma segura cuando cierras la GUI"""
        self.running = False
        self.wait()