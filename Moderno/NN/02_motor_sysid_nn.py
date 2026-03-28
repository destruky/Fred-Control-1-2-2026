"""
FrED Control Moderno — Identificación de Motor DC con Red Neuronal
Corre esto: python 01_motor_sysid_nn.py
"""
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import scipy.io
from pathlib import Path

# ==============================================================
# CONFIG
# ==============================================================
PWM_MAX   = 255.0
RPM_MAX   = 55.0
WINDOW    = 5
HIDDEN    = 32
EPOCHS    = 150
LR        = 1e-3
BATCH     = 256
Ts        = 0.1

BASE       = Path(__file__).parent
FILE_TRAIN = BASE / "PRBS_Motor1.csv"
FILE_VAL   = BASE / "PRBS_Motor2.csv"
FILE_TEST  = BASE / "PRBS_Motor3.csv"

# ==============================================================
# PASO 1: CARGAR Y LIMPIAR
# ==============================================================
def load_prbs(filepath):
    df = pd.read_csv(filepath, skiprows=1, header=None, names=['t_ms', 'pwm', 'rpm'])
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors='coerce')
    df.dropna(inplace=True)
    df.reset_index(drop=True, inplace=True)
    df['rpm'] = df['rpm'].abs()
    df['pwm_n'] = df['pwm'] / PWM_MAX
    df['rpm_n'] = df['rpm'] / RPM_MAX
    return df

def make_windows(df, window=WINDOW):
    X, Y = [], []
    pwm = df['pwm_n'].values
    rpm = df['rpm_n'].values
    for k in range(window, len(df) - 1):
        feat = np.concatenate([pwm[k - window:k + 1], rpm[k - window:k + 1]])
        target = rpm[k + 1]
        X.append(feat)
        Y.append(target)
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)

# ==============================================================
# PASO 2: RED NEURONAL
# ==============================================================
class MotorIdentifier(nn.Module):
    def __init__(self, input_dim, hidden=HIDDEN):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, 1)
        )
    def forward(self, x):
        return self.net(x).squeeze(-1)

# ==============================================================
# PASO 3: ENTRENAR
# ==============================================================
def train_model(model, train_dl, val_dl, epochs=EPOCHS, lr=LR):
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()
    best_val = float('inf')
    history = {'train': [], 'val': []}

    for epoch in range(epochs):
        model.train()
        t_loss = 0
        for xb, yb in train_dl:
            pred = model(xb)
            loss = criterion(pred, yb)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            t_loss += loss.item() * len(xb)
        t_loss /= len(train_dl.dataset)

        model.eval()
        v_loss = 0
        with torch.no_grad():
            for xb, yb in val_dl:
                v_loss += criterion(model(xb), yb).item() * len(xb)
        v_loss /= len(val_dl.dataset)

        history['train'].append(t_loss)
        history['val'].append(v_loss)

        if v_loss < best_val:
            best_val = v_loss
            torch.save(model.state_dict(), BASE / 'motor_identifier.pth')

        if (epoch + 1) % 25 == 0 or epoch == 0:
            print(f"  Epoch {epoch+1:3d}/{epochs}: train={t_loss:.6f}  val={v_loss:.6f}")

    model.load_state_dict(torch.load(BASE / 'motor_identifier.pth'))
    return history

# ==============================================================
# PASO 4: SIMULAR MULTI-STEP
# ==============================================================
def simulate_multistep(model, df, window=WINDOW):
    rpm_sim = np.zeros(len(df))
    rpm_sim[:window + 1] = df['rpm_n'].values[:window + 1]
    model.eval()
    with torch.no_grad():
        for k in range(window, len(df) - 1):
            feat = np.concatenate([
                df['pwm_n'].values[k - window:k + 1],
                rpm_sim[k - window:k + 1]
            ]).astype(np.float32)
            rpm_sim[k + 1] = model(torch.from_numpy(feat).unsqueeze(0)).item()
    return rpm_sim * RPM_MAX

# ==============================================================
# PASO 5: EXTRAER ESPACIO DE ESTADOS (JACOBIANO)
# ==============================================================
def extract_state_space(model, window=WINDOW):
    candidates = [
        (75.0, 28.0),
        (60.0, 22.0),
        (90.0, 30.0),
        (90.0, 35.0),
        (80.0, 25.0),
        (100.0, 38.0),
    ]

    all_eigs = []
    for pwm_op, rpm_op in candidates:
        pwm_n = pwm_op / PWM_MAX
        rpm_n = rpm_op / RPM_MAX

        pwm_vec = torch.full((window + 1,), pwm_n, dtype=torch.float32)
        rpm_vec = torch.full((window + 1,), rpm_n, dtype=torch.float32)

        feat = torch.cat([pwm_vec, rpm_vec]).unsqueeze(0)
        feat_var = feat.clone().detach().requires_grad_(True)
        out = model(feat_var)
        out.backward()
        grad = feat_var.grad.squeeze().numpy()

        dfdpwm = grad[:window + 1]
        dfdrpm = grad[window + 1:]

        n_states = window + 1
        A = np.zeros((n_states, n_states))
        A[0, :] = dfdrpm[::-1]
        for i in range(1, n_states):
            A[i, i - 1] = 1.0

        B = np.zeros((n_states, 1))
        B[0, 0] = dfdpwm[-1]

        C = np.zeros((1, n_states))
        C[0, 0] = 1.0

        D = np.zeros((1, 1))

        B_real = B * (RPM_MAX / PWM_MAX)

        eigs = np.abs(np.linalg.eigvals(A))
        all_eigs.append((pwm_op, rpm_op, eigs))

        if all(eigs < 1):
            print(f"  Punto de operación seleccionado: PWM={pwm_op}, RPM={rpm_op}")
            print(f"  |eigenvalues|: {eigs.round(4)}")
            return A, B_real, C, D

    print("  Ningún punto de operación produjo un sistema estable:")
    for pwm_op, rpm_op, eigs in all_eigs:
        print(f"    PWM={pwm_op}, RPM={rpm_op}: |eigs|={eigs.round(4)}")
    raise ValueError("No se encontró un punto de operación estable en ninguno de los candidatos.")

# ==============================================================
# MAIN — CORRE TODO
# ==============================================================
if __name__ == '__main__':
    print("=" * 60)
    print("FrED Control Moderno — Red Neuronal del Motor DC")
    print("=" * 60)

    # Cargar
    print("\n[1/5] Cargando datos...")
    train_df = load_prbs(FILE_TRAIN)
    val_df   = load_prbs(FILE_VAL)
    test_df  = load_prbs(FILE_TEST)
    print(f"  Train: {len(train_df)}, Val: {len(val_df)}, Test: {len(test_df)}")

    # Ventanas
    print("\n[2/5] Creando dataset...")
    X_train, Y_train = make_windows(train_df)
    X_val, Y_val     = make_windows(val_df)
    X_test, Y_test   = make_windows(test_df)

    train_dl = DataLoader(TensorDataset(torch.from_numpy(X_train), torch.from_numpy(Y_train)),
                          batch_size=BATCH, shuffle=True)
    val_dl   = DataLoader(TensorDataset(torch.from_numpy(X_val), torch.from_numpy(Y_val)),
                          batch_size=512)

    # Entrenar
    print(f"\n[3/5] Entrenando ({EPOCHS} epochs)...")
    model = MotorIdentifier(input_dim=X_train.shape[1], hidden=HIDDEN)
    history = train_model(model, train_dl, val_dl)

    # Evaluar
    print("\n[4/5] Evaluando...")
    model.eval()
    with torch.no_grad():
        pred = model(torch.from_numpy(X_test)).numpy() * RPM_MAX
        real = Y_test * RPM_MAX
    rmse = np.sqrt(np.mean((pred - real) ** 2))
    r2 = 1 - np.var(pred - real) / np.var(real)

    rpm_sim = simulate_multistep(model, test_df)
    rmse_multi = np.sqrt(np.mean((rpm_sim[WINDOW+1:] - test_df['rpm'].values[WINDOW+1:]) ** 2))

    print(f"\n  RESULTADOS:")
    print(f"  1-step:     RMSE = {rmse:.2f} RPM,  R² = {r2:.4f}")
    print(f"  Multi-step: RMSE = {rmse_multi:.2f} RPM")

    # Espacio de estados
    print("\n[5/5] Extrayendo espacio de estados...")
    A, B, C, D = extract_state_space(model)
    scipy.io.savemat(BASE / 'state_space_motor.mat', {'A': A, 'B': B, 'C': C, 'D': D, 'Ts': Ts})
    np.savez(BASE / 'state_space_motor.npz', A=A, B=B, C=C, D=D, Ts=Ts)

    print(f"\n  Matriz A:\n{np.array2string(A, precision=4)}")
    print(f"\n  Matriz B:\n{np.array2string(B, precision=4)}")

    eigs = np.abs(np.linalg.eigvals(A))
    print(f"  Sistema {'ESTABLE' if all(eigs < 1) else 'INESTABLE'}")

    print(f"\n  Archivos guardados:")
    print(f"    motor_identifier.pth  — pesos de la red")
    print(f"    state_space_motor.mat — matrices A, B, C, D") # Cambié el nombre aquí

    # ... (El código de las gráficas déjalo exactamente igual) ...

    # --- CAMBIE LAS INSTRUCCIONES FINALES PARA MATLAB ---
    print("\n✅ LISTO.")
    print("Siguiente paso: arrastra el archivo state_space_motor.mat a MATLAB o corre esto:")
    print("  >> load('state_space_motor.mat');")
    print("  >> sys_d = ss(A, B, C, D, Ts);")
    print("  >> step(sys_d)")

    # Graficar
    t_s = (test_df['t_ms'].values - test_df['t_ms'].values[0]) / 1000.0
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle('Red Neuronal vs Realidad (Motor DC)', fontsize=14, fontweight='bold')

    axes[0].plot(t_s, test_df['rpm'].values, 'r-', alpha=0.6, linewidth=0.6, label='Real')
    axes[0].plot(t_s, rpm_sim, 'b--', alpha=0.8, linewidth=0.8, label='NN Multi-step')
    axes[0].set_ylabel('|RPM|')
    axes[0].set_title(f'RMSE={rmse_multi:.2f} RPM')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    mask = t_s <= 60
    axes[1].plot(t_s[mask], test_df['rpm'].values[mask], 'r-', linewidth=1, label='Real')
    axes[1].plot(t_s[mask], rpm_sim[mask], 'b--', linewidth=1.2, label='NN')
    axes[1].set_ylabel('|RPM|')
    axes[1].set_xlabel('Tiempo (s)')
    axes[1].set_title('Zoom 0-60s')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(BASE / 'motor_nn_results.png', dpi=150)
    plt.show()

    