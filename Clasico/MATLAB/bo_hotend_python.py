"""
bo_hotend_python.py
Reemplazo del script MATLAB fred_clasico_bo_hotend.m

Hace lo mismo pero 100% en Python (no requiere Statistics Toolbox de MATLAB):
1. Carga CSVs de escalones del hotend
2. Identifica G(z) con ARX (mismo método que sysid.py)
3. Corre BO con scikit-optimize en los rangos: Kp[30,60], Ki[5,20], Kd[0.1,10]
4. Compara contra ganancias manuales del año pasado (25, 2.5, 1.5)
5. Guarda resultados en fred_bo_hotend_results.json
"""

import sys
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Asegurar que se puede importar sysid.py y bo_pid.py del Adaptativo
_THIS = Path(__file__).parent
_ADAPT = _THIS.parent / "Adaptativo"
sys.path.insert(0, str(_ADAPT))

from sysid import identify_hotend, simulate_arx
from bo_pid import optimize_pid


# ============================================================
# 1) Cargar CSVs de escalones hotend (PWM constante por tramo)
# ============================================================
DATA_DIR = _THIS.parent / "PRBS_Test" / "Info" / "Buena"

csv_files = [
    ("3)hotend60.csv",  60),
    ("1)hotend100.csv", 100),
    ("2)hotend180.csv", 180),
    ("4)hotend220.csv", 220),
]

all_u, all_y = [], []
for fname, pwm_level in csv_files:
    fpath = DATA_DIR / fname
    if not fpath.exists():
        print(f"WARN: no encontrado {fpath}")
        continue
    df = pd.read_csv(fpath)
    # Intentar varias convenciones de columnas
    cols_lower = [c.lower() for c in df.columns]
    if 'temp' in cols_lower:
        y_col = df.columns[cols_lower.index('temp')]
    elif 'temperatura' in cols_lower:
        y_col = df.columns[cols_lower.index('temperatura')]
    else:
        y_col = df.columns[-1]  # última columna como temperatura
    y_seg = df[y_col].values.astype(float)
    # Limpiar NaN
    mask = ~np.isnan(y_seg)
    y_seg = y_seg[mask]
    if len(y_seg) == 0:
        print(f"  WARN: {fname} tiene todos NaN, descartado")
        continue
    u_seg = np.full_like(y_seg, pwm_level, dtype=float)
    all_u.append(u_seg)
    all_y.append(y_seg)
    print(f"Cargado {fname}: PWM={pwm_level}, N={len(y_seg)} (tras NaN), T_final={y_seg[-1]:.1f}°C")

u = np.concatenate(all_u)
y = np.concatenate(all_y)

# Verificación final: no debe haber NaN ni Inf
mask = np.isfinite(u) & np.isfinite(y)
u = u[mask]
y = y[mask]
print(f"\nTotal muestras (limpias): {len(u)}\n")

if len(u) < 100:
    raise ValueError("Muy pocos datos válidos para identificar")


# ============================================================
# 2) Identificar G(z) del hotend
# ============================================================
print("=" * 60)
print("Identificando G(z) del hotend...")
print("=" * 60)
num, den, fit = identify_hotend(u.tolist(), y.tolist(), Ts=0.1)
print(f"num = {list(num)}")
print(f"den = {list(den)}")
print(f"FIT = {fit:.2f}%\n")


# ============================================================
# 3) Ganancias manuales del año pasado para comparar
# ============================================================
Kp_actual = 25.0
Ki_actual = 2.5
Kd_actual = 1.5
print(f"Ganancias manuales (año pasado): Kp={Kp_actual}, Ki={Ki_actual}, Kd={Kd_actual}\n")


# ============================================================
# 4) Correr BO con rangos correctos
# ============================================================
print("=" * 60)
print("Corriendo Optimización Bayesiana (rangos Kp[30,60], Ki[5,20], Kd[0.1,10])")
print("=" * 60)
result = optimize_pid(
    num, den, Ts=0.1,
    planta='hotend',
    x0=[Kp_actual, Ki_actual, Kd_actual],
    n_calls=30,
    verbose=True,
)

if not result['success']:
    print(f"BO falló: {result.get('error', 'desconocido')}")
    sys.exit(1)

Kp_bo = result['Kp']
Ki_bo = result['Ki']
Kd_bo = result['Kd']
ITAE_bo = result['itae']

print(f"\nResultado BO:")
print(f"  Kp = {Kp_bo:.4f}")
print(f"  Ki = {Ki_bo:.4f}")
print(f"  Kd = {Kd_bo:.4f}")
print(f"  ITAE = {ITAE_bo:.4f}\n")
print(f"Comando GUI: PIDH:{Kp_bo:.4f},{Ki_bo:.4f},{Kd_bo:.4f}")


# ============================================================
# 5) Guardar resultados
# ============================================================
results_file = _THIS / "fred_bo_hotend_results.json"
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
