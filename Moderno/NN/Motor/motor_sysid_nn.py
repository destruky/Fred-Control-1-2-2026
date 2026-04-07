"""
FrED Control Moderno — Identificación de Motor DC con Red Neuronal
Corre esto: python motor_sysid_nn.py
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

    model.load_state_dict(torch.load(BASE / 'motor_identifier.pth', weights_only=True))
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
# PASO 5: SANITY CHECK
# Verifica que el modelo tiene sentido físico antes de extraer matrices.
# ==============================================================
def sanity_check(model, window=WINDOW):
    print("\n  --- Sanity Check ---")
    ok = True

    # Subir PWM debe subir predicción de RPM
    tests = [
        (30,  5,  "PWM=30,  RPM=5   → debe subir"),
        (60,  20, "PWM=60,  RPM=20  → debe subir"),
        (90,  35, "PWM=90,  RPM=35  → cerca de SS, delta≈0"),
        (120, 45, "PWM=120, RPM=45  → debe subir o mantenerse"),
    ]

    model.eval()
    for pwm, rpm, desc in tests:
        feat = torch.cat([
            torch.full((window + 1,), pwm / PWM_MAX),
            torch.full((window + 1,), rpm / RPM_MAX)
        ]).unsqueeze(0)
        with torch.no_grad():
            pred = model(feat).item() * RPM_MAX
        delta = pred - rpm
        status = "OK" if delta >= -1.0 else "FAIL"
        if delta < -2.0:
            ok = False
        print(f"    {desc}: pred={pred:.2f} RPM (Δ={delta:+.3f}) [{status}]")

    # Gradiente total de PWM debe ser positivo
    feat = torch.cat([
        torch.full((window + 1,), 75 / PWM_MAX),
        torch.full((window + 1,), 28 / RPM_MAX)
    ]).unsqueeze(0).requires_grad_(True)
    out = model(feat)
    out.backward()
    grad = feat.grad.squeeze().numpy()
    dfdpwm = grad[:window + 1]
    dfdrpm = grad[window + 1:]

    scale = RPM_MAX / PWM_MAX
    sum_pwm = dfdpwm.sum()
    sum_rpm  = dfdrpm.sum()
    dc_direct = sum_pwm * scale / (1.0 - sum_rpm) if abs(1 - sum_rpm) > 1e-6 else float('inf')

    print(f"    Grad (PWM=75, RPM=28):")
    print(f"      Σ(dfdpwm) = {sum_pwm:.6f} ({'OK +' if sum_pwm > 0 else 'FAIL -'})")
    print(f"      Σ(dfdrpm) = {sum_rpm:.6f} (debe ≈ 1.0)")
    print(f"      dfdpwm[-1] = {dfdpwm[-1]:.6f} ({'OK +' if dfdpwm[-1] > 0 else 'WARNING -'})")
    print(f"      DC gain directa = {dc_direct:.4f} RPM/PWM (empírica ≈ 0.3-0.5)")

    if sum_pwm < 0:
        print(f"      WARNING: Σ(dfdpwm) negativo → modelo físicamente incorrecto")
        ok = False
    if dfdpwm[-1] < 0:
        print(f"      WARNING: dfdpwm[-1] negativo → B[0,0] tendrá signo incorrecto en companion form simple")
    if dc_direct < 0:
        ok = False

    print(f"  Sanity check: {'PASSED' if ok else 'FAILED'}")
    return ok

# ==============================================================
# PASO 6: EXTRAER ESPACIO DE ESTADOS (JACOBIANO)
#
# Companion form COMPLETA con estados de PWM delay.
#
# El NARX tiene: rpm_n(k+1) = f(pwm_n(k-W..k), rpm_n(k-W..k))
# La forma anterior solo tenía estados de RPM → los W+1 gradientes
# de PWM se comprimían en un solo B[0,0] = dfdpwm[-1], perdiendo
# los W gradientes restantes.
#
# Forma corregida:
#   x(k) = [rpm(k), ..., rpm(k-W), pwm(k-1), ..., pwm(k-W)]
#   dim(x) = (W+1) + W = 2W+1 = 11 estados
#   u(k) = pwm(k)
#   y(k) = rpm(k) = C*x
#
# En unidades físicas (RPM para rpm, PWM 0-255 para pwm/input):
#   A[0, 0:W+1]    = dfdrpm[::-1]             (adimensional)
#   A[0, W+1:2W+1] = dfdpwm[:W][::-1] * scale (PWM→RPM)
#   B[0]   = dfdpwm[W] * scale                (pwm(k) → rpm(k+1))
#   B[W+1] = 1                                (pwm(k) entra al delay chain)
#
# DC gain = Σ(dfdpwm) * scale / (1 - Σ(dfdrpm))
#         ≈ 0.3-0.5 RPM/PWM para el motor del FrED
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
    scale = RPM_MAX / PWM_MAX

    all_results = []
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

        dfdpwm = grad[:window + 1]   # ∂f/∂pwm(k-W), ..., ∂f/∂pwm(k)
        dfdrpm = grad[window + 1:]   # ∂f/∂rpm(k-W), ..., ∂f/∂rpm(k)

        # --- Companion form completa: 2W+1 estados ---
        n = 2 * window + 1

        A = np.zeros((n, n))
        # Fila 0: dinámica de rpm(k+1)
        A[0, :window + 1] = dfdrpm[::-1]
        A[0, window + 1:] = dfdpwm[window - 1::-1] * scale

        # Shift register de RPM
        for i in range(1, window + 1):
            A[i, i - 1] = 1.0

        # Shift register de PWM
        for j in range(window + 2, n):
            A[j, j - 1] = 1.0

        B = np.zeros((n, 1))
        B[0, 0]          = dfdpwm[window] * scale  # pwm(k) → rpm(k+1)
        B[window + 1, 0] = 1.0                      # pwm(k) → delay chain

        C = np.zeros((1, n))
        C[0, 0] = 1.0   # y = rpm(k)

        D = np.zeros((1, 1))

        # --- Verificaciones ---
        eigs = np.linalg.eigvals(A)
        eigs_abs = np.abs(eigs)
        stable = all(eigs_abs < 1)

        sum_pwm = dfdpwm.sum()
        sum_rpm  = dfdrpm.sum()
        dc_direct = sum_pwm * scale / (1.0 - sum_rpm) if abs(1 - sum_rpm) > 1e-6 else float('inf')
        dc_matrix = (C @ np.linalg.solve(np.eye(n) - A, B)).item()

        all_results.append({
            'pwm': pwm_op, 'rpm': rpm_op,
            'eigs_max': eigs_abs.max(), 'stable': stable,
            'dc_direct': dc_direct, 'dc_matrix': dc_matrix,
            'A': A, 'B': B, 'C': C, 'D': D,
            'sum_pwm': sum_pwm, 'sum_rpm': sum_rpm,
        })

        print(f"  ({pwm_op:.0f} PWM, {rpm_op:.0f} RPM): "
              f"|eigs|_max={eigs_abs.max():.4f}, "
              f"DC={dc_direct:+.4f} RPM/PWM, "
              f"Σpwm={sum_pwm:+.4f}, "
              f"{'ESTABLE' if stable else 'inestable'}")

    # Seleccionar: estable + DC positivo + DC más cercano al empírico (~0.4)
    valid = [r for r in all_results if r['stable'] and r['dc_direct'] > 0]
    if not valid:
        # Fallback: companion form original W+1 estados con warning
        print("\n  WARNING: companion form completa no produjo sistema estable+DC positivo.")
        print("  Usando fallback: companion form original W+1 estados.")
        return _extract_state_space_fallback(model, window, candidates)

    best = min(valid, key=lambda r: abs(r['dc_direct'] - 0.4))

    print(f"\n  SELECCIONADO: PWM={best['pwm']:.0f}, RPM={best['rpm']:.0f}")
    print(f"    |eigs| max = {best['eigs_max']:.4f}")
    print(f"    DC gain (directa)   = {best['dc_direct']:.4f} RPM/PWM")
    print(f"    DC gain (C(I-A)⁻¹B) = {best['dc_matrix']:.4f} RPM/PWM")
    print(f"    Σ(dfdpwm)={best['sum_pwm']:.6f}, Σ(dfdrpm)={best['sum_rpm']:.6f}")
    print(f"    A: {best['A'].shape}, B: {best['B'].shape}")

    return best['A'], best['B'], best['C'], best['D']


def _extract_state_space_fallback(model, window, candidates):
    """Companion form original W+1 estados — fallback si la completa falla."""
    scale = RPM_MAX / PWM_MAX
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

        n = window + 1
        A = np.zeros((n, n))
        A[0, :] = dfdrpm[::-1]
        for i in range(1, n):
            A[i, i - 1] = 1.0

        B = np.zeros((n, 1))
        B[0, 0] = dfdpwm[-1] * scale

        C = np.zeros((1, n))
        C[0, 0] = 1.0

        D = np.zeros((1, 1))

        eigs = np.abs(np.linalg.eigvals(A))
        if all(eigs < 1):
            print(f"  Fallback seleccionado: PWM={pwm_op}, RPM={rpm_op}, "
                  f"|eigs|_max={eigs.max():.4f}")
            return A, B, C, D

    raise ValueError("No se encontró punto de operación estable en ningún candidato.")


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

    # Sanity check
    model_ok = sanity_check(model)

    # Espacio de estados
    print("\n[5/5] Extrayendo espacio de estados...")
    A, B, C, D = extract_state_space(model)

    scipy.io.savemat(BASE / 'state_space_motor.mat', {'A': A, 'B': B, 'C': C, 'D': D, 'Ts': Ts})
    np.savez(BASE / 'state_space_motor.npz', A=A, B=B, C=C, D=D, Ts=Ts)

    print(f"\n  Matriz A ({A.shape[0]}x{A.shape[1]}):\n{np.array2string(A, precision=4)}")
    print(f"\n  Matriz B:\n{np.array2string(B, precision=4)}")

    eigs = np.abs(np.linalg.eigvals(A))
    print(f"  |eigs| max: {eigs.max():.4f}")
    print(f"  Sistema {'ESTABLE' if all(eigs < 1) else 'INESTABLE'}")

    print(f"\n  Archivos guardados:")
    print(f"    motor_identifier.pth  — pesos de la red")
    print(f"    state_space_motor.mat — matrices A, B, C, D")

    print("\n✅ LISTO.")
    print("Siguiente paso en MATLAB:")
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
