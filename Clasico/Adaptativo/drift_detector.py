"""
drift_detector.py — Detección de drift entre planta real y modelo G(s).

Compara la salida real (y_real) contra la simulación del modelo actual
(y_pred) ante la misma entrada u. Si el residuo normalizado supera un
umbral, dispara re-identificación + BO.

Uso:
    detector = DriftDetector(threshold_pct=15.0)
    has_drift, residual = detector.check(num, den, u_window, y_window)
"""

import numpy as np
from sysid import simulate_arx, predict_one_step


class DriftDetector:
    """
    Detector de drift basado en residuo normalizado.

    Métrica:  NRMSE = ||y_real - y_pred|| / ||y_real - mean(y_real)|| · 100%

    Si NRMSE > threshold_pct, el modelo ya no representa la planta y se
    debe re-identificar.

    Args:
        threshold_pct: umbral de drift en porcentaje (default 15%)
        min_samples: mínimo de muestras para evaluar
        d: retardo del modelo ARX (igual al usado en identify_arx)
    """

    def __init__(self, threshold_pct=15.0, min_samples=50, d=1):
        self.threshold_pct = threshold_pct
        self.min_samples = min_samples
        self.d = d
        self.last_residual_pct = None

    def check(self, num, den, u_window, y_window):
        """
        Compara modelo (num, den) contra ventana de datos reales.

        Args:
            num, den: coeficientes ARX del modelo actual
            u_window: array PWM aplicados (entrada)
            y_window: array salida medida (RPM o °C)

        Returns:
            (has_drift: bool, residual_pct: float)
        """
        u = np.asarray(u_window, dtype=float).ravel()
        y = np.asarray(y_window, dtype=float).ravel()

        if len(u) != len(y):
            return False, np.nan
        if len(u) < self.min_samples:
            return False, np.nan

        try:
            # Predicción a 1 paso (usa y medida real como historia). NO usar
            # simulate_arx (free-run): arranca desde y=0 y para un sistema
            # casi-integrador como el hotend da residuos de ~15000% espurios.
            y_pred = predict_one_step(num, den, u, y, d=self.d)
        except Exception:
            return False, np.nan

        # Saltamos las primeras muestras transitorias del simulador
        skip = max(len(num), len(den), self.d) + 5
        if len(y) - skip < 10:
            return False, np.nan

        y_real = y[skip:]
        y_hat = y_pred[skip:]

        mask = np.isfinite(y_hat)
        if not np.any(mask):
            return False, np.nan

        y_real = y_real[mask]
        y_hat = y_hat[mask]

        ref = np.linalg.norm(y_real - np.mean(y_real))
        if ref < 1e-9:
            return False, 0.0

        err = np.linalg.norm(y_real - y_hat)
        nrmse_pct = 100.0 * err / ref

        self.last_residual_pct = nrmse_pct
        has_drift = nrmse_pct > self.threshold_pct

        return has_drift, nrmse_pct

    def reset(self):
        self.last_residual_pct = None


if __name__ == "__main__":
    # Test rápido
    np.random.seed(0)
    u = np.random.uniform(60, 120, 200)
    # "planta real" con coef [0, 0.05, 0.02] / [1, -1.6, 0.65]
    y_real = simulate_arx([0, 0.05, 0.02], [1, -1.6, 0.65], u, d=1)

    # Modelo bueno (mismo)
    det = DriftDetector(threshold_pct=15.0)
    drift, res = det.check([0, 0.05, 0.02], [1, -1.6, 0.65], u, y_real)
    print(f"Modelo correcto: drift={drift}, residuo={res:.2f}%")

    # Modelo malo (otro)
    drift, res = det.check([0, 0.1, 0.01], [1, -1.0, 0.3], u, y_real)
    print(f"Modelo erróneo: drift={drift}, residuo={res:.2f}%")
