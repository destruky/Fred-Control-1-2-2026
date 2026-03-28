import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import numpy as np
import scipy.io as sio
import os
from sklearn.metrics import root_mean_squared_error, r2_score
from sklearn.model_selection import train_test_split

# =========================================================================
# 1. PREPARACIÓN DE DATOS (Identificación Global)
# =========================================================================
PWM_MAX = 255.0
TEMP_MAX = 250.0
W = 5  # Ventana de estados pasados (5 de temp, 5 de pwm)

def load_and_window(filepath, W):
    """Carga un CSV, normaliza y crea ventanas sin mezclar archivos."""
    if not os.path.exists(filepath):
        print(f"⚠️ Archivo no encontrado: {filepath}")
        return None, None
        
    df = pd.read_csv(filepath).dropna()
    # Normalización basada en límites físicos reales
    df['pwm_norm'] = df['pwm_heater'].astype(float) / PWM_MAX
    df['temp_norm'] = df['temp_C'].astype(float) / TEMP_MAX
    
    X, Y = [], []
    u = df['pwm_norm'].values
    y = df['temp_norm'].values
    
    # Creamos el historial de estados para el espacio de estados
    for i in range(W, len(df) - 1):
        u_window = u[i-W : i+1] # PWM actual y pasados
        y_window = y[i-W : i+1] # Temp actual y pasadas
        # El vector de entrada es [y(k), y(k-1)... u(k), u(k-1)...]
        X.append(np.concatenate((y_window[::-1], u_window[::-1])))
        Y.append(y[i+1]) # El siguiente estado de temperatura
        
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32).reshape(-1, 1)

# Rutas automáticas
script_dir = os.path.dirname(os.path.abspath(__file__))
base_path = os.path.join(script_dir, "../Datos CSV/Datos para entrenar/")

# Cargamos los 4 experimentos para que la IA vea TODO el rango
archivos = ["3)hotend60.csv", "1)hotend100.csv", "2)hotend180.csv", "4)hotend220.csv"]
X_all, Y_all = [], []

print("📂 Cargando y procesando los 4 datasets de FrED...")
for arc in archivos:
    path = os.path.join(base_path, arc)
    x_tmp, y_tmp = load_and_window(path, W)
    if x_tmp is not None:
        X_all.append(x_tmp)
        Y_all.append(y_tmp)

# Concatenamos todo en un solo cerebro de datos
X_final = np.vstack(X_all)
Y_final = np.vstack(Y_all)

# Dividimos en entrenamiento y validación (80% / 20%) de forma aleatoria
X_train, X_test, Y_train, Y_test = train_test_split(X_final, Y_final, test_size=0.2, random_state=42)

# Convertir a tensores de PyTorch
X_train_t = torch.tensor(X_train)
Y_train_t = torch.tensor(Y_train)
X_test_t = torch.tensor(X_test)
Y_test_t = torch.tensor(Y_test)

# =========================================================================
# 2. ARQUITECTURA Y ENTRENAMIENTO NN (El Cerebro del Sistema)
# =========================================================================
class HotendNet(nn.Module):
    def __init__(self, W):
        super(HotendNet, self).__init__()
        input_size = 2 * (W + 1) # y(k..k-5) y u(k..k-5)
        self.net = nn.Sequential(
            nn.Linear(input_size, 32),
            nn.ReLU(),
            nn.Linear(32, 32),
            nn.ReLU(),
            nn.Linear(32, 1)
        )

    def forward(self, x):
        return self.net(x)

model = HotendNet(W)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.0005)

print(f"\n🚀 Iniciando entrenamiento global ({len(X_train)} muestras)...")
epochs = 1000
for epoch in range(epochs):
    model.train()
    optimizer.zero_grad()
    outputs = model(X_train_t)
    loss = criterion(outputs, Y_train_t)
    loss.backward()
    optimizer.step()
    
    if (epoch+1) % 100 == 0:
        print(f"Epoch [{epoch+1}/{epochs}], MSE Loss: {loss.item():.8f}")

# Evaluación final
model.eval()
with torch.no_grad():
    preds_real = model(X_test_t).numpy() * TEMP_MAX
    Y_test_real = Y_test * TEMP_MAX

rmse = root_mean_squared_error(Y_test_real, preds_real)
r2 = r2_score(Y_test_real, preds_real)
print(f"\n🎯 RESULTADO FINAL -> RMSE: {rmse:.4f} °C, R²: {r2:.4f}")

# =========================================================================
# 3. LINEALIZACIÓN (Extracción de Matrices A, B, C, D)
# =========================================================================
# Punto de operación nominal: 150°C
op_u = np.full(W+1, 130.0 / PWM_MAX, dtype=np.float32)
op_y = np.full(W+1, 150.0 / TEMP_MAX, dtype=np.float32)
op_point = torch.tensor(np.concatenate((op_y, op_u)), requires_grad=True)

out = model(op_point.unsqueeze(0))
out.backward()
jacobian = op_point.grad.numpy()

# Separar derivadas parciales respecto a Y y U
dy = jacobian[0:W+1]
du = jacobian[W+1:]

# Construcción de la forma canónica de espacio de estados
state_dim = 2*W + 1
A = np.zeros((state_dim, state_dim))
B = np.zeros((state_dim, 1))
C = np.zeros((1, state_dim))
D = np.zeros((1, 1))

# Matriz A: Dinámica de los estados
A[0, 0:W+1] = dy
B[0, 0] = du[0]

# Relaciones de retraso (Shift registers térmicos)
for i in range(1, W+1):
    A[i, i-1] = 1
for i in range(W+1, state_dim-1):
    A[i+1, i] = 1

C[0, 0] = 1 # La salida es la temperatura actual

# Escalamiento de unidades para MATLAB/Simulink (°C / PWM)
B_real = B * (TEMP_MAX / PWM_MAX)

# Comprobación de estabilidad (Crucial para que Simulink no explote)
max_eig = np.max(np.abs(np.linalg.eigvals(A)))
print(f"\nPolos del sistema (|λ|): {max_eig:.6f}")

# =========================================================================
# 4. GUARDAR RESULTADOS
# =========================================================================
sio.savemat(os.path.join(script_dir, "state_space_hotend.mat"), 
            {'A': A, 'B': B_real, 'C': C, 'D': D, 'Ts': 0.1})

print("\n" + "="*50)
print("✅ MODELO LISTO PARA MATLAB Y SIMULINK")
print(f"Archivo guardado en: {os.path.join(script_dir, 'state_space_hotend.mat')}")
print("="*50)