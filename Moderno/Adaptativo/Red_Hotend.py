"""
FrED Control Moderno — Identificación de Hotend con Red Neuronal
MLP-NARX + Two-Phase Training (1-step warmup → NOE curriculum) + tanh

CAMBIOS v3:
  FIX: TEMP_MAX=225 (rango real datos, no 250/255)
  FIX: WINDOW=100 (10s ≈ 10% de τ, W=30 solo cubría 0.2°C de señal)
  FIX: Two-phase training — Phase1 aprende dinámica local correcta,
       Phase2 la estabiliza sobre horizonte largo. v2 iba directo a NOE
       H=100 y el modelo aprendía a copiar temp ignorando PWM.
  FIX: ganancia DC usa (C @ M).item()
  FIX: Sanity check antes de extraer matrices — si multi-step RMSE
       es muy alto o DC gain es negativo, no guarda matrices basura.
  ADD: Diagnóstico completo del modelo (gradient check, DC gain sanity)

Ref: Hotend-NN-Multi-Step-Prediction-Investigacion-y-Guia-de-Implementacion.md
Corre esto: python hotend_sysid_nn.py
"""
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import scipy.io as sio
from torch.utils.data import DataLoader, TensorDataset
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from pathlib import Path

# ==============================================================
# CONFIG
# ==============================================================
PWM_MAX    = 255.0
TEMP_MAX   = 225.0     # Rango real: hotend220 llega a 225°C
WINDOW     = 100       # 10s de historia ≈ 10% de τ=100s
HIDDEN     = 128       # Más capacidad para input_dim=202
Ts         = 0.1

# Phase 1: 1-step warmup
P1_EPOCHS  = 200
P1_LR      = 1e-3
P1_BATCH   = 256

# Phase 2: Multi-step NOE con curriculum
P2_EPOCHS  = 400
P2_LR      = 3e-4      # Menor LR para no destruir lo aprendido en Phase 1
P2_H_MIN   = 20        # Curriculum: empezar casi 1-step
P2_H_MAX   = 300       # Curriculum: 30s = 30% de τ
P2_TBPTT   = 50        # Detach cada 50 pasos
P2_BATCH   = 16
P2_TF_START = 0.5
P2_TF_END   = 0.0

VAL_FRAC   = 0.2       # 20% de cada archivo para validación

BASE = Path(__file__).parent
# Los datos y pesos viven en NN/Hotend/ (dos niveles arriba desde Adaptativo/)
DATA_BASE = BASE.parent / "NN" / "Hotend"

# 3 archivos train/val, hotend220 exclusivo para test
TRAIN_FILES = [
    DATA_BASE / "3)hotend60.csv",
    DATA_BASE / "1)hotend100.csv",
    DATA_BASE / "2)hotend180.csv",
]
TEST_FILE = DATA_BASE / "4)hotend220.csv"

# ==============================================================
# PASO 1: CARGAR Y LIMPIAR
# ==============================================================
def load_hotend(filepath):
    df = pd.read_csv(filepath)
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors='coerce')
    df.dropna(inplace=True)
    df.reset_index(drop=True, inplace=True)
    df['pwm_n']  = df['pwm_heater'] / PWM_MAX
    df['temp_n'] = df['temp_C'] / TEMP_MAX
    return df

# ==============================================================
# PASO 2: DATASET
# ==============================================================
def make_windows(df, window=WINDOW):
    """Ventanas individuales [pwm(k-W..k), temp(k-W..k)] → temp(k+1)."""
    X, Y = [], []
    pwm  = df['pwm_n'].values
    temp = df['temp_n'].values
    for k in range(window, len(df) - 1):
        feat = np.concatenate([pwm[k - window:k + 1], temp[k - window:k + 1]])
        X.append(feat)
        Y.append(temp[k + 1])
    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)

def make_subsequences(df, window, h_max):
    """Subsecuencias de largo window + h_max + 1 para NOE training."""
    T = window + h_max + 1
    stride = max(T // 4, 1)
    pwm  = df['pwm_n'].values
    temp = df['temp_n'].values
    starts = list(range(0, len(df) - T, stride))
    if len(starts) == 0:
        return np.zeros((0, T), dtype=np.float32), np.zeros((0, T), dtype=np.float32)
    seqs_pwm  = np.array([pwm[s:s + T]  for s in starts], dtype=np.float32)
    seqs_temp = np.array([temp[s:s + T] for s in starts], dtype=np.float32)
    return seqs_pwm, seqs_temp

# ==============================================================
# PASO 3: RED NEURONAL (tanh para sysid NARX)
# ==============================================================
class HotendIdentifier(nn.Module):
    def __init__(self, input_dim, hidden=HIDDEN):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1)
        )
    def forward(self, x):
        return self.net(x).squeeze(-1)

# Alias para compatibilidad con Adaptativo_Worker.py
HotendNARX = HotendIdentifier

# ==============================================================
# PASO 4: PHASE 1 — 1-STEP WARMUP
# Objetivo: aprender dinámica local correcta antes de multi-step.
# ==============================================================
def train_phase1(model, train_dl, val_dl, epochs=P1_EPOCHS, lr=P1_LR):
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.MSELoss()
    best_val  = float('inf')
    history   = {'train': [], 'val': []}

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

        v_rmse = np.sqrt(v_loss) * TEMP_MAX
        history['train'].append(t_loss)
        history['val'].append(v_rmse)

        if v_rmse < best_val:
            best_val = v_rmse
            torch.save(model.state_dict(), DATA_BASE / 'hotend_identifier.pth')

        if (epoch + 1) % 25 == 0 or epoch == 0:
            print(f"  P1 Epoch {epoch+1:3d}/{epochs}: "
                  f"train_mse={t_loss:.8f}  val_rmse={v_rmse:.4f}°C  "
                  f"best={best_val:.4f}°C")

    model.load_state_dict(torch.load(DATA_BASE / 'hotend_identifier.pth'))
    print(f"  Phase 1 done: best val RMSE = {best_val:.4f}°C")
    return history

# ==============================================================
# PASO 5: PHASE 2 — MULTI-STEP NOE CON CURRICULUM
# ==============================================================
def teacher_force_prob(epoch, total_epochs):
    return P2_TF_START - (P2_TF_START - P2_TF_END) * min(epoch / total_epochs, 1.0)

def get_curriculum_horizon(epoch, total_epochs):
    """Horizonte crece de H_MIN a H_MAX en 60% del training."""
    progress = min(epoch / (total_epochs * 0.6), 1.0)
    return int(P2_H_MIN + (P2_H_MAX - P2_H_MIN) * progress)

def multistep_loss(model, pwm_batch, temp_batch, window, horizon,
                   p_tf=0.0, tbptt_k2=0):
    """
    Multi-step loss NOE.
    Feat: [pwm(k-W..k), temp(k-W..k)] — idéntico a simulate_multistep.
    Per-sample TF con torch.where.
    TBPTT: detach cada k2 pasos.
    """
    B = pwm_batch.shape[0]
    temp_pred = temp_batch[:, :window + 1].clone()
    total_loss = 0.0

    for n in range(horizon):
        k = window + n
        feat_pwm = pwm_batch[:, k - window: k + 1]

        feat_temp_pred = temp_pred[:, k - window: k + 1]
        if p_tf > 0:
            feat_temp_real = temp_batch[:, k - window: k + 1]
            mask = (torch.rand(B, 1) < p_tf)
            feat_temp = torch.where(mask, feat_temp_real, feat_temp_pred)
        else:
            feat_temp = feat_temp_pred

        feat  = torch.cat([feat_pwm, feat_temp], dim=1)
        y_hat = model(feat)
        total_loss += F.mse_loss(y_hat, temp_batch[:, k + 1])

        temp_pred = torch.cat([temp_pred, y_hat.unsqueeze(1)], dim=1)

        if tbptt_k2 > 0 and (n + 1) % tbptt_k2 == 0:
            temp_pred = temp_pred.detach()

    return total_loss / horizon

def train_phase2(model, train_dl, val_dl, diag_df, epochs=P2_EPOCHS, lr=P2_LR):
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    best_val  = float('inf')
    patience  = 0
    prev_h    = 0
    history   = {'train': [], 'val_rmse': [], 'horizon': []}

    for epoch in range(epochs):
        h_curr = get_curriculum_horizon(epoch, epochs)

        if h_curr > prev_h:
            best_val = float('inf')
            patience = 0
            prev_h = h_curr

        model.train()
        p_tf = teacher_force_prob(epoch, epochs)
        t_loss, n_b = 0.0, 0

        for pwm_b, temp_b in train_dl:
            optimizer.zero_grad()
            loss = multistep_loss(
                model, pwm_b, temp_b, WINDOW, h_curr, p_tf, P2_TBPTT)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimizer.step()
            t_loss += loss.item()
            n_b += 1
        t_loss /= n_b

        model.eval()
        v_loss, v_b = 0.0, 0
        with torch.no_grad():
            for pwm_b, temp_b in val_dl:
                v_loss += multistep_loss(
                    model, pwm_b, temp_b, WINDOW, h_curr, p_tf=0.0
                ).item()
                v_b += 1
        v_loss /= v_b
        v_rmse = np.sqrt(v_loss) * TEMP_MAX

        history['train'].append(t_loss)
        history['val_rmse'].append(v_rmse)
        history['horizon'].append(h_curr)

        if v_rmse < best_val:
            best_val = v_rmse
            patience = 0
            torch.save(model.state_dict(), DATA_BASE / 'hotend_identifier.pth')
        else:
            patience += 1

        if (epoch + 1) % 25 == 0 or epoch == 0:
            print(f"  P2 Epoch {epoch+1:3d}/{epochs}: "
                  f"h={h_curr:3d}  train_mse={t_loss:.8f}  "
                  f"val_rmse={v_rmse:.2f}°C  p_tf={p_tf:.2f}  "
                  f"best={best_val:.2f}°C")

        if (epoch + 1) % 50 == 0 and diag_df is not None:
            sim_diag = simulate_multistep(model, diag_df, verbose=False)
            real_d = diag_df['temp_C'].values[WINDOW + 1:]
            sim_d  = sim_diag[WINDOW + 1:]
            diag_rmse = np.sqrt(np.mean((sim_d - real_d) ** 2))
            diag_ss   = abs(np.mean(sim_d[-100:]) - np.mean(real_d[-100:]))
            print(f"    >> Diag full-file: "
                  f"RMSE={diag_rmse:.2f}°C, SS offset={diag_ss:.2f}°C")

        if patience >= 60:
            print(f"  Early stop epoch {epoch+1} (patience={patience})")
            break

    model.load_state_dict(torch.load(DATA_BASE / 'hotend_identifier.pth'))
    print(f"  Phase 2 done: best val RMSE = {best_val:.2f}°C (h={prev_h})")
    return history

# ==============================================================
# PASO 6: SIMULACIÓN MULTI-STEP
# Feat: [pwm(k-W..k), temp_sim(k-W..k)] — idéntico a training.
# ==============================================================
@torch.no_grad()
def simulate_multistep(model, df, window=WINDOW, verbose=True):
    model.eval()
    pwm_n  = torch.tensor(df['pwm_n'].values, dtype=torch.float32)
    temp_n = torch.tensor(df['temp_n'].values, dtype=torch.float32)

    T = len(df)
    temp_sim = torch.zeros(T)
    temp_sim[:window + 1] = temp_n[:window + 1]

    for k in range(window, T - 1):
        feat = torch.cat([
            pwm_n[k - window: k + 1],
            temp_sim[k - window: k + 1]
        ]).unsqueeze(0)
        temp_sim[k + 1] = model(feat).item()

    result = temp_sim.numpy() * TEMP_MAX

    if verbose:
        print(f"    T={T}, window={window}")
        print(f"    temp_sim[{window}] = {temp_sim[window].item()*TEMP_MAX:.1f}°C")
        print(f"    pwm = {pwm_n[0].item()*PWM_MAX:.0f} PWM")
        n_show = min(10, T - window - 2)
        idx = slice(window + 1, window + 1 + n_show)
        real_str = np.array2string(df['temp_C'].values[idx], precision=1, max_line_width=200)
        sim_str  = np.array2string(result[idx], precision=1, max_line_width=200)
        print(f"    First {n_show}: Real={real_str}")
        print(f"    First {n_show}: Sim ={sim_str}")
        print(f"    Final: real={df['temp_C'].values[-1]:.1f}°C, "
              f"sim={result[-1]:.1f}°C")

    return result

# ==============================================================
# PASO 7: DIAGNÓSTICO DEL MODELO
# Verifica gradientes del Jacobiano completo (suma total,
# no solo dfdpwm[-1]) para comprobar sentido físico.
# ==============================================================
def sanity_check(model, window=WINDOW):
    """Verifica que el modelo tiene sentido físico."""
    print("\n  --- Sanity Check ---")
    ok = True

    # Test: subir PWM debe subir predicción
    tests = [
        (60,  30, "PWM=60,  T=30°C  → debe subir"),
        (100, 50, "PWM=100, T=50°C  → debe subir"),
        (180, 100,"PWM=180, T=100°C → debe subir"),
        (180, 190,"PWM=180, T=190°C → cerca de SS, delta≈0"),
    ]

    model.eval()
    for pwm, temp, desc in tests:
        feat = torch.cat([
            torch.full((window + 1,), pwm / PWM_MAX),
            torch.full((window + 1,), temp / TEMP_MAX)
        ]).unsqueeze(0)
        with torch.no_grad():
            pred = model(feat).item() * TEMP_MAX
        delta = pred - temp
        status = "OK" if delta >= -0.5 else "FAIL"
        if delta < -1.0:
            ok = False
        print(f"    {desc}: pred={pred:.2f}°C (Δ={delta:+.3f}°C) [{status}]")

    # Test: gradient TOTAL de PWM debe ser positivo
    feat = torch.cat([
        torch.full((window + 1,), 100 / PWM_MAX),
        torch.full((window + 1,), 80 / TEMP_MAX)
    ]).unsqueeze(0).requires_grad_(True)
    out = model(feat)
    out.backward()
    grad = feat.grad.squeeze().numpy()
    dfdpwm  = grad[:window + 1]
    dfdtemp = grad[window + 1:]

    sum_pwm  = dfdpwm.sum()
    sum_temp = dfdtemp.sum()
    scale = TEMP_MAX / PWM_MAX
    # Ganancia DC directa del Jacobiano (sin construir matrices)
    dc_direct = sum_pwm * scale / (1.0 - sum_temp) if abs(1 - sum_temp) > 1e-6 else float('inf')

    print(f"    Grad (PWM=100, T=80°C):")
    print(f"      Σ(dfdpwm) = {sum_pwm:.6f} ({'OK +' if sum_pwm > 0 else 'FAIL -'})")
    print(f"      Σ(dfdtemp) = {sum_temp:.6f} (debe ≈ 1.0)")
    print(f"      dfdpwm[-1] (solo pwm(k)) = {dfdpwm[-1]:.6f}")
    print(f"      DC gain directa = {dc_direct:.4f} °C/PWM (empírica ≈ 0.9-1.1)")
    if sum_pwm < 0:
        print(f"      WARNING: Σ(dfdpwm) negativo → modelo físicamente incorrecto")
        ok = False
    if dc_direct < 0:
        ok = False

    print(f"  Sanity check: {'PASSED' if ok else 'FAILED'}")
    return ok

# ==============================================================
# PASO 8: ESPACIO DE ESTADOS (JACOBIANO)
#
# Companion form COMPLETA con estados de PWM delay.
#
# El NARX tiene: temp_n(k+1) = f(pwm_n(k-W..k), temp_n(k-W..k))
# La forma anterior solo tenía estados de temp → los W+1 gradientes
# de PWM se comprimían en un solo B[0,0] = dfdpwm[-1], perdiendo
# los W gradientes restantes (que sumaban positivo pero el último
# era negativo, causando DC gain < 0).
#
# Forma corregida:
#   x(k) = [temp(k), ..., temp(k-W), pwm(k-1), ..., pwm(k-W)]
#   dim(x) = (W+1) + W = 2W+1
#   u(k) = pwm(k)
#   y(k) = temp(k) = C*x
#
# En unidades físicas (°C para temp, PWM 0-255 para pwm/input):
#   A[0, 0:W+1]     = dfdtemp[::-1]              (°C→°C, adimensional)
#   A[0, W+1:2W+1]  = dfdpwm[:W][::-1] * scale   (PWM→°C)
#   B[0]   = dfdpwm[W] * scale                    (PWM→°C, entrada actual)
#   B[W+1] = 1                                    (pwm(k) entra al delay chain)
#
# DC gain = Σ(dfdpwm) * scale / (1 - Σ(dfdtemp))
#         ≈ 0.9-1.1 °C/PWM para el hotend del FrED
# ==============================================================
def extract_state_space(model, window=WINDOW, pwm_op=None, temp_op=None):
    # Si se pasa un punto de operacion especifico, usarlo solo
    # Si no, iterar candidatos (modo standalone/entrenamiento)
    if pwm_op is not None and temp_op is not None:
        candidates = [(pwm_op, temp_op)]
    else:
        candidates = [
            (100.0, 100.0),
            (130.0, 150.0),
            (180.0, 190.0),
            (150.0, 160.0),
            (100.0,  80.0),
            (60.0,   65.0),
        ]
    scale = TEMP_MAX / PWM_MAX

    all_results = []
    for pwm_op, temp_op in candidates:
        pwm_n  = pwm_op / PWM_MAX
        temp_n = temp_op / TEMP_MAX

        pwm_vec  = torch.full((window + 1,), pwm_n, dtype=torch.float32)
        temp_vec = torch.full((window + 1,), temp_n, dtype=torch.float32)

        feat = torch.cat([pwm_vec, temp_vec]).unsqueeze(0)
        feat_var = feat.clone().detach().requires_grad_(True)
        out = model(feat_var)
        out.backward()
        grad = feat_var.grad.squeeze().numpy()

        dfdpwm  = grad[:window + 1]    # ∂f/∂pwm(k-W), ..., ∂f/∂pwm(k)
        dfdtemp = grad[window + 1:]    # ∂f/∂temp(k-W), ..., ∂f/∂temp(k)

        # --- Companion form completa: 2W+1 estados ---
        n = 2 * window + 1

        A = np.zeros((n, n))
        # Fila 0: dinámica de temp(k+1)
        # Cols 0..W: dependencia de temp(k), temp(k-1), ..., temp(k-W)
        A[0, :window + 1] = dfdtemp[::-1]
        # Cols W+1..2W: dependencia de pwm(k-1), pwm(k-2), ..., pwm(k-W)
        A[0, window + 1:] = dfdpwm[window - 1::-1] * scale

        # Shift register de temperatura: temp(k-i+1) → temp(k-i)
        for i in range(1, window + 1):
            A[i, i - 1] = 1.0

        # Shift register de PWM: pwm(k-j+1) → pwm(k-j)
        # (fila W+1 queda en ceros — se llena desde B)
        for j in range(window + 2, n):
            A[j, j - 1] = 1.0

        B = np.zeros((n, 1))
        B[0, 0]          = dfdpwm[window] * scale   # pwm(k) → temp(k+1)
        B[window + 1, 0] = 1.0                       # pwm(k) → delay chain

        C = np.zeros((1, n))
        C[0, 0] = 1.0          # y = temp(k)

        D = np.zeros((1, 1))

        # --- Verificaciones ---
        eigs = np.linalg.eigvals(A)
        eigs_abs = np.abs(eigs)
        stable = all(eigs_abs < 1)

        # DC gain directa (debe coincidir con C(I-A)^{-1}B)
        sum_pwm  = dfdpwm.sum()
        sum_temp = dfdtemp.sum()
        dc_direct = sum_pwm * scale / (1.0 - sum_temp)

        # DC gain via matrices
        dc_matrix = (C @ np.linalg.solve(np.eye(n) - A, B)).item()

        all_results.append({
            'pwm': pwm_op, 'temp': temp_op,
            'eigs_max': eigs_abs.max(), 'stable': stable,
            'dc_direct': dc_direct, 'dc_matrix': dc_matrix,
            'A': A, 'B': B, 'C': C, 'D': D,
            'sum_pwm': sum_pwm, 'sum_temp': sum_temp,
        })

        print(f"  ({pwm_op:.0f}, {temp_op:.0f}°C): "
              f"|eigs|_max={eigs_abs.max():.4f}, "
              f"DC={dc_direct:+.4f} °C/PWM, "
              f"Σpwm={sum_pwm:+.4f}, "
              f"{'ESTABLE' if stable else 'inestable'}")

    # Seleccionar el mejor: estable + DC positivo + DC más cercano a empírico (~1)
    valid = [r for r in all_results if r['stable'] and r['dc_direct'] > 0]
    if not valid:
        # Fallback: al menos estable
        valid = [r for r in all_results if r['stable']]
    if not valid:
        print("\n  ERROR: Ningún punto estable encontrado.")
        for r in all_results:
            print(f"    ({r['pwm']:.0f}, {r['temp']:.0f}°C): "
                  f"|eigs|={r['eigs_max']:.4f}, DC={r['dc_direct']:.4f}")
        raise ValueError("No se encontró punto de operación estable.")

    # Elegir el que tenga DC gain más cercano al empírico (~1.0 °C/PWM)
    best = min(valid, key=lambda r: abs(r['dc_direct'] - 1.0))

    print(f"\n  SELECCIONADO: PWM={best['pwm']:.0f}, Temp={best['temp']:.0f}°C")
    print(f"    |eigs| max = {best['eigs_max']:.4f}")
    print(f"    DC gain (directa)  = {best['dc_direct']:.4f} °C/PWM")
    print(f"    DC gain (C(I-A)⁻¹B) = {best['dc_matrix']:.4f} °C/PWM")
    print(f"    Σ(dfdpwm)={best['sum_pwm']:.6f}, Σ(dfdtemp)={best['sum_temp']:.6f}")
    print(f"    A: {best['A'].shape}, B: {best['B'].shape}")

    return best['A'], best['B'], best['C'], best['D']

# ==============================================================
# MAIN
# ==============================================================
def obtener_matrices_11_estados(modelo, ventana_datos):
    """
    Linealiza la red en el punto de operacion ACTUAL (media de ventana_datos)
    y devuelve A, B, C, D de 2*W+1 estados via Jacobiano.
    ventana_datos: lista de WINDOW listas [[temp, pwm], ...]
    """
    modelo.eval()

    # Punto de operacion real: media de los ultimos WINDOW datos
    temps = [d[0] for d in ventana_datos]
    pwms  = [d[1] for d in ventana_datos]
    pwm_op  = float(np.mean(pwms))
    temp_op = float(np.mean(temps))

    A, B, C, D = extract_state_space(modelo, pwm_op=pwm_op, temp_op=temp_op)

    return A, B, C, D

if __name__ == "__main__":
    print("=" * 60)
    print("FrED Control Moderno — Red Neuronal del Hotend")
    print(f"W={WINDOW} ({WINDOW*Ts:.0f}s), TEMP_MAX={TEMP_MAX}, "
        f"Two-Phase Training")
    print("=" * 60)

    # ---- 1. Cargar ----
    print("\n[1/6] Cargando datos...")
    train_dfs = [load_hotend(f) for f in TRAIN_FILES]
    test_df   = load_hotend(TEST_FILE)

    for f, df in zip(TRAIN_FILES, train_dfs):
        t_range = f"{df['temp_C'].min():.1f}–{df['temp_C'].max():.1f}"
        print(f"  TRAIN {f.name}: {len(df)} rows, "
            f"PWM={df['pwm_heater'].iloc[0]:.0f}, Temp=[{t_range}]°C")
    print(f"  TEST  {TEST_FILE.name}: {len(test_df)} rows")

    # ---- 2. Datasets ----
    print(f"\n[2/6] Creando datasets (W={WINDOW})...")

    # Phase 1: windowed (X, Y) pairs
    X_tr, Y_tr = [], []
    X_va, Y_va = [], []
    for df in train_dfs:
        X_tmp, Y_tmp = make_windows(df)
        cut = int(len(X_tmp) * (1 - VAL_FRAC))
        X_tr.append(X_tmp[:cut]);  Y_tr.append(Y_tmp[:cut])
        X_va.append(X_tmp[cut:]);  Y_va.append(Y_tmp[cut:])

    X_tr = np.vstack(X_tr); Y_tr = np.concatenate(Y_tr)
    X_va = np.vstack(X_va); Y_va = np.concatenate(Y_va)

    p1_train_dl = DataLoader(
        TensorDataset(torch.from_numpy(X_tr), torch.from_numpy(Y_tr)),
        batch_size=P1_BATCH, shuffle=True)
    p1_val_dl = DataLoader(
        TensorDataset(torch.from_numpy(X_va), torch.from_numpy(Y_va)),
        batch_size=512)

    print(f"  Phase 1: {len(X_tr)} train, {len(X_va)} val windows")
    print(f"  Input dim: {X_tr.shape[1]}")

    # Phase 2: subsequences (strided split: 1/5 to val)
    train_p_list, train_t_list = [], []
    val_p_list,   val_t_list   = [], []
    for df in train_dfs:
        p_all, t_all = make_subsequences(df, WINDOW, P2_H_MAX)
        for i in range(len(p_all)):
            if i % 5 == 0:
                val_p_list.append(p_all[i])
                val_t_list.append(t_all[i])
            else:
                train_p_list.append(p_all[i])
                train_t_list.append(t_all[i])

    train_pwm  = np.array(train_p_list)
    train_temp = np.array(train_t_list)
    val_pwm    = np.array(val_p_list)
    val_temp   = np.array(val_t_list)

    p2_train_dl = DataLoader(
        TensorDataset(torch.from_numpy(train_pwm), torch.from_numpy(train_temp)),
        batch_size=P2_BATCH, shuffle=True)
    p2_val_dl = DataLoader(
        TensorDataset(torch.from_numpy(val_pwm), torch.from_numpy(val_temp)),
        batch_size=P2_BATCH)

    seq_len = WINDOW + P2_H_MAX + 1
    print(f"  Phase 2: {len(train_pwm)} train, {len(val_pwm)} val subseqs "
        f"(length={seq_len}, {seq_len*Ts:.0f}s)")

    # ---- 3. Phase 1: 1-step warmup ----
    input_dim = 2 * (WINDOW + 1)
    model = HotendIdentifier(input_dim=input_dim, hidden=HIDDEN)

    print(f"\n[3/6] Phase 1 — 1-step warmup ({P1_EPOCHS} epochs, "
        f"lr={P1_LR})...")
    h1 = train_phase1(model, p1_train_dl, p1_val_dl)

    # Quick sanity after Phase 1
    print("\n  Post-Phase 1 sanity:")
    for f, df in zip(TRAIN_FILES[:1], train_dfs[:1]):
        sim = simulate_multistep(model, df, verbose=True)
        rmse = np.sqrt(np.mean(
            (sim[WINDOW+1:] - df['temp_C'].values[WINDOW+1:]) ** 2))
        print(f"  Full-file RMSE ({f.name}): {rmse:.2f}°C")

    # ---- 4. Phase 2: Multi-step NOE ----
    print(f"\n[4/6] Phase 2 — NOE curriculum ({P2_EPOCHS} epochs, "
        f"H={P2_H_MIN}→{P2_H_MAX}, lr={P2_LR})...")
    h2 = train_phase2(model, p2_train_dl, p2_val_dl, diag_df=train_dfs[2])

    # ---- 5. Evaluar ----
    print("\n[5/6] Evaluando multi-step...")
    model.eval()

    all_files = TRAIN_FILES + [TEST_FILE]
    all_dfs   = train_dfs   + [test_df]
    labels    = ['TRAIN'] * len(TRAIN_FILES) + ['TEST']

    rmse_results = {}
    for f, df, label in zip(all_files, all_dfs, labels):
        print(f"\n  --- [{label}] {f.name} ---")
        temp_sim_i = simulate_multistep(model, df, verbose=True)
        real_i = df['temp_C'].values[WINDOW + 1:]
        sim_i  = temp_sim_i[WINDOW + 1:]
        rmse_i = np.sqrt(np.mean((sim_i - real_i) ** 2))
        r2_i   = 1 - np.var(sim_i - real_i) / np.var(real_i)
        ss_offset = abs(np.mean(sim_i[-100:]) - np.mean(real_i[-100:]))
        print(f"  RMSE={rmse_i:.2f}°C, R²={r2_i:.4f}, "
            f"SS offset={ss_offset:.2f}°C")
        rmse_results[f.name] = rmse_i

    # 1-step para comparación
    with torch.no_grad():
        pred_1s = model(torch.from_numpy(X_va)).numpy() * TEMP_MAX
        real_1s = Y_va * TEMP_MAX
    rmse_1s = np.sqrt(np.mean((pred_1s - real_1s) ** 2))
    r2_1s   = 1 - np.var(pred_1s - real_1s) / np.var(real_1s)
    print(f"\n  1-step (val): RMSE={rmse_1s:.4f}°C, R²={r2_1s:.4f}")

    # ---- 6. Espacio de estados (solo si sanity check pasa) ----
    print("\n[6/6] Espacio de estados...")
    model_ok = sanity_check(model)

    # Check multi-step quality
    train_rmse_avg = np.mean([rmse_results[f.name] for f in TRAIN_FILES])
    if train_rmse_avg > 30:
        print(f"\n  WARNING: RMSE promedio train = {train_rmse_avg:.1f}°C > 30°C")
        print(f"  Las matrices A,B,C,D NO serán confiables.")
        print(f"  Se guardan de todas formas pero revisar antes de usar en LQR.")

    try:
        A, B, C, D = extract_state_space(model)

        np.savez(DATA_BASE / 'state_space_hotend.npz', A=A, B=B, C=C, D=D, Ts=Ts)
        sio.savemat(str(DATA_BASE / 'state_space_hotend.mat'), {
            'A': A, 'B': B, 'C': C, 'D': D, 'Ts': Ts})

        n_st = A.shape[0]
        n_temp = WINDOW + 1
        n_pwm  = WINDOW
        print(f"\n  A: {n_st}x{n_st} ({n_temp} estados temp + {n_pwm} estados PWM)")
        print(f"  B[0,0] = {B[0,0]:.6f} °C/PWM (pwm(k) → temp(k+1))")
        print(f"  B[{n_temp},0] = {B[n_temp,0]:.1f} (pwm(k) → delay chain)")
        eigs = np.abs(np.linalg.eigvals(A))
        print(f"  |eigs| max: {eigs.max():.4f}")
        print(f"  Sistema {'ESTABLE' if all(eigs < 1) else 'INESTABLE'}")

        print(f"\n  Archivos guardados:")
        print(f"    hotend_identifier.pth  — pesos de la red")
        print(f"    state_space_hotend.npz — A,B,C,D (Python)")
        print(f"    state_space_hotend.mat — A,B,C,D (MATLAB)")
        print(f"\n  NOTA: {n_st} estados es demasiado para Arduino.")
        print(f"  En MATLAB usar minreal() o balred() para reducir a ~5-10 estados.")
    except ValueError as e:
        print(f"\n  ERROR extrayendo matrices: {e}")
        print(f"  Solo se guardó hotend_identifier.pth")

    # ---- Gráficas ----
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f'Red Neuronal vs Realidad (Hotend) — W={WINDOW}, Two-Phase',
                fontsize=14, fontweight='bold')

    for idx, (f, df, label) in enumerate(zip(all_files, all_dfs, labels)):
        ax = axes[idx // 2][idx % 2]
        t_s = (df['t_ms'].values - df['t_ms'].values[0]) / 1000.0
        temp_sim_i = simulate_multistep(model, df, verbose=False)
        rmse_i = np.sqrt(np.mean(
            (temp_sim_i[WINDOW + 1:] - df['temp_C'].values[WINDOW + 1:]) ** 2))

        tag = ' [TEST]' if label == 'TEST' else ''
        ax.plot(t_s, df['temp_C'].values, 'r-', alpha=0.7, lw=0.8, label='Real')
        ax.plot(t_s, temp_sim_i, 'b--', alpha=0.8, lw=0.8, label='NN')
        ax.set_title(f'{f.name}{tag} (RMSE={rmse_i:.2f}°C)')
        ax.set_ylabel('Temperatura (°C)')
        ax.set_xlabel('Tiempo (s)')
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(DATA_BASE / 'hotend_nn_results.png', dpi=150)
    plt.show()

    print("\nLISTO.")