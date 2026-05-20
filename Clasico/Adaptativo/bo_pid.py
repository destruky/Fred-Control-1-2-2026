"""
bo_pid.py — Optimización Bayesiana de PID con warm start.

Equivalente Python de motor_bayes_opt_2.m / fred_clasico_bo_hotend.m,
adaptado para uso en línea desde el worker adaptativo.

Funciones:
    optimize_pid(num, den, Ts, x0, n_calls, ranges, planta) -> dict

Usa scikit-optimize (skopt). Warm start vía x0 garantiza que el resultado
sea al menos tan bueno como el PID actual.
"""

import numpy as np
from scipy import signal

try:
    from skopt import gp_minimize
    from skopt.space import Real
except ImportError:
    raise ImportError(
        "scikit-optimize no está instalado. Ejecuta:\n"
        "    pip install scikit-optimize"
    )


# ----------------------------------------------------------------------
# Simulador de lazo cerrado discreto con PID + saturación + anti-windup
# ----------------------------------------------------------------------

def simulate_closed_loop(num, den, Ts, Kp, Ki, Kd,
                         setpoint, T_end, pwm_min=0, pwm_max=255,
                         deadzone=0, meas_noise_std=0.0):
    """
    Simula la respuesta al escalón de la planta discreta con PID.

    Args:
        num, den: coeficientes de G(z) discretos
        Ts: tiempo de muestreo
        Kp, Ki, Kd: ganancias PID
        setpoint: referencia (valor constante)
        T_end: duración total de simulación (s)
        pwm_min, pwm_max: límites de saturación
        deadzone: PWM mínimo efectivo (motor: 30, hotend: 0)

    Returns:
        t, y, u (vectores de tiempo, salida, control)
    """
    N = int(T_end / Ts) + 1
    t = np.arange(N) * Ts

    # Convertir a state-space
    A, B, C, D = signal.tf2ss(num, den)
    nx = A.shape[0]
    x = np.zeros((nx,))

    y = np.zeros(N)
    u = np.zeros(N)

    e_int = 0.0
    e_prev = 0.0
    d_filter = 0.0
    Tf = 5 * Ts  # filtro derivativo de primer orden (N=5)

    # Ruido de medición del sensor (encoder/termistor). Seed fijo para que el
    # BO vea un objetivo reproducible. Penaliza ganancias agresivas de forma
    # física: un Kp/Kd alto amplifica el ruido y degrada el ITAE real.
    if meas_noise_std > 0:
        noise = np.random.default_rng(42).normal(0.0, meas_noise_std, N)
    else:
        noise = np.zeros(N)

    for k in range(N):
        e = setpoint - (y[k] + noise[k])

        # Derivativa filtrada
        d_raw = (e - e_prev) / Ts
        d_filter = (Tf * d_filter + Kd * d_raw) / (Tf + Ts)

        u_raw = Kp * e + Ki * e_int + d_filter

        # Zona muerta: si el control raw está dentro de la zona muerta efectiva, sale 0
        if deadzone > 0 and 0 < u_raw < deadzone:
            u_raw = deadzone

        u_sat = np.clip(u_raw, pwm_min, pwm_max)

        # Anti-windup condicional
        if u_raw == u_sat:
            e_int += e * Ts

        e_prev = e
        u[k] = u_sat

        # Avanzar planta
        if k < N - 1:
            x = A @ x + B.ravel() * u_sat
            y[k + 1] = float(C @ x + D.ravel() * u_sat)

    return t, y, u


# ----------------------------------------------------------------------
# Función objetivo ITAE
# ----------------------------------------------------------------------

def compute_itae(num, den, Ts, Kp, Ki, Kd, setpoint, T_end,
                 pwm_min=0, pwm_max=255, deadzone=0,
                 overshoot_penalty=0, max_safe=None, kp_penalty=0.0,
                 sse_penalty=0.0, meas_noise_std=0.0):
    """
    Calcula ITAE = sum(k·|e(k)|·Ts) con penalizaciones opcionales.

    overshoot_penalty: peso por °C/RPM por encima del setpoint
    max_safe: si la salida supera este valor, devuelve 1e6 (cutoff)
    kp_penalty: penalización fraccional por Kp alto. El simulador es sin ruido,
                pero en hardware un Kp alto amplifica el ruido del sensor y
                genera ciclo límite. itae *= (1 + kp_penalty·Kp) sesga el BO
                hacia ganancias más suaves (menos oscilación real).
    """
    try:
        t, y, _ = simulate_closed_loop(
            num, den, Ts, Kp, Ki, Kd,
            setpoint, T_end, pwm_min, pwm_max, deadzone, meas_noise_std
        )

        if max_safe is not None and np.max(y) > max_safe:
            return 1e6

        if not np.all(np.isfinite(y)):
            return 1e6

        e = setpoint - y
        itae = np.trapz(np.abs(e) * t, dx=Ts)

        if overshoot_penalty > 0:
            overshoot = max(0, np.max(y) - setpoint)
            itae += overshoot_penalty * overshoot

        if kp_penalty > 0:
            itae *= (1.0 + kp_penalty * Kp)

        # Penalizar error en estado estacionario (último 20% de la simulación).
        # El modelo lineal predice SSE=0 teórico para cualquier Ki>0, pero en
        # la práctica Ki bajo → integrador lento → SSE real alto. Esta penalización
        # fuerza al BO a preferir Ki suficientemente alto para eliminar el offset.
        if sse_penalty > 0:
            final_idx = int(0.8 * len(t))
            sse = float(np.mean(np.abs(setpoint - y[final_idx:])))
            itae += sse_penalty * sse

        return float(itae) if np.isfinite(itae) else 1e6
    except Exception:
        return 1e6


# ----------------------------------------------------------------------
# Optimización Bayesiana
# ----------------------------------------------------------------------

def optimize_pid(num, den, Ts, planta='motor', x0=None, n_calls=10,
                 ranges=None, verbose=False):
    """
    Optimiza PID minimizando ITAE con BO.

    Args:
        num, den: G(z) discreto identificado
        Ts: tiempo de muestreo
        planta: 'motor' o 'hotend' (define setpoint, T_end, rangos default)
        x0: warm start [Kp_actual, Ki_actual, Kd_actual]
        n_calls: número de evaluaciones BO (con warm start, 8–12 basta)
        ranges: dict opcional con 'Kp', 'Ki', 'Kd' como tuplas (min, max)
        verbose: imprime progreso

    Returns:
        dict con Kp, Ki, Kd, itae, n_calls, success
    """
    # Configuración por planta
    if planta == 'motor':
        setpoint = 55.0  # rango real del motor con mecanismo: ~50-63 RPM (no alcanza 20 por stiction)
        T_end = 5.0   # motor rápido (τ≈0.08s): 5s hace que el transitorio pese en el ITAE
        pwm_min = 120  # zona muerta motor con mecanismo
        pwm_max = 255
        deadzone = 120
        # Rango amplio para que el BO pueda alcanzar (y superar) la zona del
        # PID Tuner (Ki~58). Si el rango excluye esa región el BO no puede
        # competir. Kd acotado: el encoder amplifica ruido con Kd grande.
        default_ranges = {'Kp': (0.1, 30.0), 'Ki': (0.01, 100.0), 'Kd': (0.0, 2.0)}
        overshoot_penalty = 0
        max_safe = None
        kp_penalty = 0.0
        sse_penalty = 0.0
        meas_noise_std = 0.4   # ruido del encoder (RPM) — penaliza Kp/Kd agresivos
    elif planta == 'hotend':
        setpoint = 200.0
        T_end = 800.0
        pwm_min = 0          # el hotend NO tiene zona muerta (PWM=0 apaga el heater)
        pwm_max = 255
        deadzone = 0
        # Ki acotado a la zona analíticamente estable (ζ>=1): para el modelo
        # térmico (K~1, τ~159s) Ki>~4 produce ciclo límite. Rango previo
        # [0.05, 1.5] dejaba SSE de ~2°C; tope subido a 3.0 para cerrar el
        # offset sin reentrar en oscilación (kp_penalty + overshoot_penalty
        # evitan zonas inestables).
        default_ranges = {'Kp': (35.0, 65.0), 'Ki': (0.5, 4.0), 'Kd': (0.05, 3.0)}
        overshoot_penalty = 500
        max_safe = 250.0
        kp_penalty = 0.005
        sse_penalty = 50_000  # fuerza Ki suficientemente alto para cerrar el offset
        meas_noise_std = 0.0   # termistor tiene ruido bajo; hotend ya usa sse_penalty
    else:
        raise ValueError(f"planta debe ser 'motor' o 'hotend', no {planta}")

    if ranges is None:
        ranges = default_ranges

    space = [
        Real(ranges['Kp'][0], ranges['Kp'][1], name='Kp'),
        Real(ranges['Ki'][0], ranges['Ki'][1], name='Ki'),
        Real(ranges['Kd'][0], ranges['Kd'][1], name='Kd'),
    ]

    def objective(params):
        Kp, Ki, Kd = params
        return compute_itae(
            num, den, Ts, Kp, Ki, Kd, setpoint, T_end,
            pwm_min, pwm_max, deadzone, overshoot_penalty, max_safe, kp_penalty,
            sse_penalty, meas_noise_std
        )

    # Warm start: forzar la primera evaluación en x0 si está dentro de rangos
    x0_list = None
    if x0 is not None:
        x0_clamped = [
            float(np.clip(x0[0], ranges['Kp'][0], ranges['Kp'][1])),
            float(np.clip(x0[1], ranges['Ki'][0], ranges['Ki'][1])),
            float(np.clip(x0[2], ranges['Kd'][0], ranges['Kd'][1])),
        ]
        x0_list = [x0_clamped]

    try:
        # Si hay warm start, contamos el x0 como "initial point" y sólo
        # exploramos n_calls-1 con la función de adquisición.
        # Si no, dejamos los defaults de skopt (10 random + resto BO).
        if x0_list is not None:
            n_initial = 1
            n_calls_eff = max(n_calls, n_initial + 5)
        else:
            n_initial = min(5, n_calls // 2)
            n_calls_eff = max(n_calls, n_initial + 5)

        result = gp_minimize(
            objective,
            space,
            n_calls=n_calls_eff,
            n_initial_points=n_initial,
            x0=x0_list,
            acq_func='EI',
            random_state=42,
            verbose=verbose,
        )

        best = result.x
        return {
            'Kp': float(best[0]),
            'Ki': float(best[1]),
            'Kd': float(best[2]),
            'itae': float(result.fun),
            'n_calls': n_calls,
            'success': True,
        }
    except Exception as e:
        return {
            'Kp': x0[0] if x0 else None,
            'Ki': x0[1] if x0 else None,
            'Kd': x0[2] if x0 else None,
            'itae': float('inf'),
            'n_calls': 0,
            'success': False,
            'error': str(e),
        }


# ----------------------------------------------------------------------
# Test manual
# ----------------------------------------------------------------------

if __name__ == "__main__":
    # Ejemplo: motor de prueba (segundo orden)
    num = [0.0, 0.05, 0.02]
    den = [1.0, -1.6, 0.65]
    Ts = 0.1

    print("Ejecutando BO motor con warm start [25, 2.5, 1.5]...")
    res = optimize_pid(num, den, Ts, planta='motor',
                       x0=[25, 2.5, 1.5], n_calls=10, verbose=True)
    print(f"Resultado: Kp={res['Kp']:.4f}, Ki={res['Ki']:.4f}, "
          f"Kd={res['Kd']:.4f}, ITAE={res['itae']:.4f}")
