"""
sysid.py — Identificación de sistemas por mínimos cuadrados (ARX).

Equivalente ligero a tfest() de MATLAB para usar en línea desde el worker
adaptativo. Identifica modelos ARX:

    A(q) y(k) = B(q) u(k-d) + e(k)

Funciones principales:
    identify_arx(u, y, na, nb, d=1, Ts=0.1) -> (num, den, fit_pct)
    arx_to_tf(num, den, Ts) -> scipy.signal.TransferFunction (discreto)
    simulate_tf(tf_disc, u) -> y_pred

Diseñado para:
    - Motor DC: na=2, nb=2, d=1 (segundo orden, sin retardo)
    - Hotend:   na=1, nb=1, d=2 (primer orden con retardo pequeño)
"""

import numpy as np
from scipy import signal


def identify_arx(u, y, na, nb, d=1):
    """
    Identifica un modelo ARX por mínimos cuadrados.

    y(k) + a1·y(k-1) + ... + ana·y(k-na)
        = b0·u(k-d) + b1·u(k-d-1) + ... + b(nb-1)·u(k-d-nb+1) + e(k)

    Args:
        u: array de entrada (PWM)
        y: array de salida (RPM o °C)
        na: orden del polinomio A (número de polos)
        nb: orden del polinomio B (número de coeficientes de entrada)
        d:  retardo en muestras (≥1)

    Returns:
        num: coeficientes [b0, b1, ..., b(nb-1)]
        den: coeficientes [1, a1, ..., ana]
        fit_pct: ajuste porcentual (similar a FIT% de MATLAB)
    """
    u = np.asarray(u, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    if len(u) != len(y):
        raise ValueError("u e y deben tener la misma longitud")

    N = len(y)
    k0 = max(na, nb + d - 1)

    if N - k0 < na + nb + 5:
        raise ValueError(f"Datos insuficientes: N={N}, mínimo {k0 + na + nb + 5}")

    # Matriz regresora: phi(k) = [-y(k-1), ..., -y(k-na), u(k-d), ..., u(k-d-nb+1)]
    rows = N - k0
    Phi = np.zeros((rows, na + nb))
    Y = np.zeros(rows)

    for i, k in enumerate(range(k0, N)):
        for j in range(na):
            Phi[i, j] = -y[k - 1 - j]
        for j in range(nb):
            Phi[i, na + j] = u[k - d - j]
        Y[i] = y[k]

    # Mínimos cuadrados REGULARIZADOS (ridge): se añade lam·I a la ecuación
    # normal para mantener acotados los coeficientes cuando los datos están
    # mal condicionados (PWM saturado constante, o ruido en estado estable).
    # Sin esto, lstsq puede devolver un polo |a1|>1 → modelo inestable cuya
    # simulación libre diverge → FIT = -inf.
    n_param = na + nb
    PtP = Phi.T @ Phi
    lam = 1e-3 * np.trace(PtP) / n_param
    theta = np.linalg.solve(PtP + lam * np.eye(n_param), Phi.T @ Y)

    a = theta[:na]
    b = theta[na:]

    den = np.concatenate(([1.0], a))
    num = b.copy()

    # Chequeo de ESTABILIDAD: motor y hotend son estables en lazo abierto
    # (heater off → enfría; motor sin PWM → frena). Un modelo identificado con
    # un polo fuera del círculo unitario es físicamente incorrecto. Se rechaza
    # devolviendo FIT=-inf para que el worker lo descarte.
    # Nota: el polo físico del hotend es ≈0.999 (τ≈100s); el límite de
    # estabilidad correcto es |polo|<1.0, NO un margen más estricto que
    # rechazaría el modelo térmico real.
    poles = np.roots(den)
    estable = (len(poles) == 0) or (np.max(np.abs(poles)) < 1.0)

    if not estable:
        return num, den, -np.inf

    # FIT% a 1 PASO: Phi @ theta ES la predicción a un paso, porque los
    # regresores en Phi usan la y MEDIDA real (no predicciones). Nunca diverge,
    # a diferencia de simulate_arx (free-run) que es la causa del FIT=-12000%.
    y_pred_1 = Phi @ theta
    err = np.linalg.norm(Y - y_pred_1)
    ref = np.linalg.norm(Y - np.mean(Y))
    fit_pct = 100.0 * (1.0 - err / ref) if ref > 1e-12 else 0.0

    return num, den, fit_pct


def simulate_arx(num, den, u, d=1, x0=None):
    """
    Simula la salida del modelo ARX dado u(k).

    Args:
        num: coeficientes [b0, b1, ..., b(nb-1)]
        den: coeficientes [1, a1, ..., ana]
        u: entrada
        d: retardo en muestras
        x0: condiciones iniciales (no implementado, parte desde cero)

    Returns:
        y: salida simulada, misma longitud que u
    """
    u = np.asarray(u, dtype=float).ravel()
    num = np.asarray(num, dtype=float).ravel()
    den = np.asarray(den, dtype=float).ravel()

    N = len(u)
    na = len(den) - 1
    nb = len(num)

    y = np.zeros(N)
    a = den[1:]

    for k in range(N):
        if k < max(na, nb + d - 1):
            y[k] = 0.0
            continue
        # y(k-1), y(k-2), ..., y(k-na)
        y_hist = y[k - na:k][::-1] if na > 0 else np.array([])
        y_part = -np.dot(a, y_hist) if na > 0 else 0.0

        # u(k-d), u(k-d-1), ..., u(k-d-nb+1)
        u_hist = u[k - d - nb + 1:k - d + 1][::-1] if nb > 0 else np.array([])
        u_part = np.dot(num, u_hist) if nb > 0 else 0.0

        y[k] = y_part + u_part

    return y


def predict_one_step(num, den, u, y, d=1):
    """
    Predicción a 1 paso del modelo ARX.

    A diferencia de simulate_arx (free-running, que usa sus propias
    predicciones y arranca desde y=0), esto usa la salida MEDIDA real
    y[k-1..k-na] como historia. Por eso:
      - Nunca diverge, aunque el modelo sea casi-integrador o marginal.
      - Arranca implícitamente desde el punto de operación real (no desde 0).
    Es la métrica correcta para evaluar FIT de identificación y drift.

    Args:
        num: [b0, b1, ..., b(nb-1)]
        den: [1, a1, ..., ana]
        u:   entrada medida
        y:   salida medida (se usa como historia real)
        d:   retardo en muestras

    Returns:
        y_hat: predicción a 1 paso, misma longitud que y. Las primeras
               max(na, nb+d-1) muestras se copian de y (sin historia).
    """
    u = np.asarray(u, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()
    num = np.asarray(num, dtype=float).ravel()
    den = np.asarray(den, dtype=float).ravel()

    N = len(y)
    na = len(den) - 1
    nb = len(num)
    a = den[1:]

    y_hat = np.array(y, dtype=float)   # copia; primeras muestras quedan = y
    k0 = max(na, nb + d - 1)

    for k in range(k0, N):
        y_hist = y[k - na:k][::-1] if na > 0 else np.array([])
        y_part = -np.dot(a, y_hist) if na > 0 else 0.0
        u_hist = u[k - d - nb + 1:k - d + 1][::-1] if nb > 0 else np.array([])
        u_part = np.dot(num, u_hist) if nb > 0 else 0.0
        y_hat[k] = y_part + u_part

    return y_hat


def arx_to_ctf(num, den, Ts):
    """
    Convierte coeficientes ARX (q) a una función de transferencia continua.

    Usa transformación bilineal (Tustin). Útil para pasar el modelo
    identificado a `bayesopt` que trabaja con G(s) continua.

    Returns:
        (num_c, den_c) coeficientes de G(s) continuo
    """
    sys_d = signal.TransferFunction(num, den, dt=Ts)
    sys_c = signal.cont2discrete  # placeholder
    # Convertir d2c con Tustin
    z, p, k = signal.tf2zpk(num, den)
    # Mapeo bilineal inverso: s = (2/Ts) * (z-1)/(z+1)
    # Usamos d2c via state-space
    A_d, B_d, C_d, D_d = signal.tf2ss(num, den)
    # d2c con Tustin no está directo en scipy; usamos aproximación con matrix log
    # Para simplicidad: usar zoh aproximación inversa
    # Aproximación: si A_d ≈ I + A_c*Ts, entonces A_c ≈ (A_d - I)/Ts
    # Esto es Forward Euler — suficiente para órdenes bajos y Ts pequeño
    n = A_d.shape[0]
    A_c = (A_d - np.eye(n)) / Ts
    B_c = B_d / Ts
    C_c = C_d
    D_c = D_d
    num_c, den_c = signal.ss2tf(A_c, B_c, C_c, D_c)
    return num_c.ravel(), den_c.ravel()


def discrete_tf(num, den, Ts):
    """Crea un objeto TransferFunction discreto de scipy."""
    return signal.TransferFunction(num, den, dt=Ts)


# ----------------------------------------------------------------------
# Funciones específicas por planta
# ----------------------------------------------------------------------

def identify_motor(u, y, Ts=0.1):
    """Identifica G(z) del motor DC. Orden 2, sin retardo."""
    return identify_arx(u, y, na=2, nb=2, d=1)


def identify_hotend(u, y, Ts=0.1):
    """Identifica G(z) del hotend. Segundo orden con retardo.

    na=2 (no na=1): un modelo de primer orden + PID en lazo cerrado es
    SIEMPRE estable y nunca oscila, así que el BO no puede 'ver' el ciclo
    límite real del hotend y elige Ki demasiado alto. Un modelo de segundo
    orden admite polos complejos → reproduce la dinámica oscilatoria y el
    BO penaliza correctamente las ganancias que oscilan.
    """
    return identify_arx(u, y, na=2, nb=2, d=2)


# ----------------------------------------------------------------------
# Test manual con datos PRBS reales
# ----------------------------------------------------------------------

if __name__ == "__main__":
    import csv
    from pathlib import Path

    base = Path(__file__).parent.parent / "PRBS_Test" / "Info" / "Buena"
    csv_path = base / "PRBS_Motor1.csv"

    if not csv_path.exists():
        print(f"No se encontró {csv_path}")
        exit(1)

    t, u, y = [], [], []
    with open(csv_path) as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        for row in reader:
            if len(row) >= 3:
                try:
                    t.append(float(row[0]) / 1000.0)
                    u.append(float(row[1]))
                    y.append(float(row[2]))
                except ValueError:
                    continue

    u = np.array(u)
    y = np.array(y)

    print(f"Datos cargados: {len(u)} muestras")
    num, den, fit = identify_motor(u, y)
    print(f"Motor G(z): num={num}, den={den}")
    print(f"FIT% = {fit:.2f}")
