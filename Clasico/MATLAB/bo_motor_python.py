"""
bo_motor_python.py
Bayesian Optimization para PID del Motor DC — equivalente Python de motor_bayes_opt_2.m
"""

import sys
from pathlib import Path
import json
import numpy as np
import pandas as pd

_THIS = Path(__file__).parent
_ADAPT = _THIS.parent / "Adaptativo"
sys.path.insert(0, str(_ADAPT))

from sysid import identify_motor, simulate_arx
from bo_pid import optimize_pid

# ============================================================
# 1) Cargar CSV PRBS del motor (TRAIN)
# ============================================================
fpath = _THIS.parent / "Resultados" / "CSVs" / "PRBSmotorfinal.csv"

df = pd.read_csv(fpath)
cols = [c.lower() for c in df.columns]

if 'pwm' in cols:
    u_col = df.columns[cols.index('pwm')]
elif 'pwm_motor' in cols:
    u_col = df.columns[cols.index('pwm_motor')]
else:
    u_col = df.columns[1]

if 'rpm' in cols:
    y_col = df.columns[cols.index('rpm')]
else:
    y_col = df.columns[2]

u = df[u_col].values.astype(float)
y = df[y_col].values.astype(float)

# Limpiar
mask = np.isfinite(u) & np.isfinite(y)
u, y = u[mask], y[mask]
print(f"Datos cargados: {len(u)} muestras de {fpath.name}")
print(f"PWM: {u.min():.0f}-{u.max():.0f} | RPM: {y.min():.1f}-{y.max():.1f}\n")

if len(u) < 50:
    raise ValueError("Muy pocos datos")

# ============================================================
# 2) Identificar G(z) del motor
# ============================================================
print("=" * 60)
print("Identificando G(z) del motor...")
print("=" * 60)
num, den, fit = identify_motor(u.tolist(), y.tolist())
print(f"num = {list(num)}")
print(f"den = {list(den)}")
print(f"FIT = {fit:.2f}%\n")

# ============================================================
# 3) Ganancias manuales para warm start
# ============================================================
Kp_actual, Ki_actual, Kd_actual = 1.8, 0.9, 0.3
print(f"Ganancias manuales: Kp={Kp_actual}, Ki={Ki_actual}, Kd={Kd_actual}\n")

# ============================================================
# 4) Correr BO
# ============================================================
print("=" * 60)
print("Corriendo BO motor (rangos Kp[0.1,10], Ki[0.01,10], Kd[0,10])")
print("=" * 60)
result = optimize_pid(
    num, den, Ts=0.1,
    planta='motor',
    x0=[Kp_actual, Ki_actual, Kd_actual],
    n_calls=30,
    verbose=True,
)

if not result['success']:
    print(f"BO falló: {result.get('error')}")
    sys.exit(1)

Kp_bo = result['Kp']
Ki_bo = result['Ki']
Kd_bo = result['Kd']
ITAE_bo = result['itae']

print(f"\nResultado BO:")
print(f"  Kp = {Kp_bo:.4f}")
print(f"  Ki = {Ki_bo:.4f}")
print(f"  Kd = {Kd_bo:.4f}")
print(f"  ITAE = {ITAE_bo:.4f}")
print(f"\nComando GUI: PIDM:{Kp_bo:.4f},{Ki_bo:.4f},{Kd_bo:.4f}")

# ============================================================
# 5) Guardar resultados
# ============================================================
results_file = _THIS / "fred_bo_motor_results.json"
with open(results_file, 'w') as f:
    json.dump({
        'Kp_bo': float(Kp_bo),
        'Ki_bo': float(Ki_bo),
        'Kd_bo': float(Kd_bo),
        'ITAE_bo': float(ITAE_bo),
        'Kp_actual': Kp_actual,
        'Ki_actual': Ki_actual,
        'Kd_actual': Kd_actual,
        'num': [float(x) for x in num],
        'den': [float(x) for x in den],
        'FIT_identification_pct': float(fit),
    }, f, indent=2)
print(f"\nResultados guardados en: {results_file}")
