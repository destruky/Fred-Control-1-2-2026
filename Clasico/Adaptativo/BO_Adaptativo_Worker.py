"""
BO_Adaptativo_Worker.py — QThread del módulo adaptativo Clásico.

Análogo a Moderno/Adaptativo/Adaptativo_Worker.py pero con BO+tfest
en lugar de NN+LQR.

Ciclo:
    1. Recibe ventana de datos recientes (u, y) desde la GUI.
    2. Detecta drift entre el modelo G(z) actual y los datos.
    3. Si hay drift → re-identifica G(z) con sysid.identify_arx().
    4. Corre BO con warm start desde Kp/Ki/Kd actuales.
    5. Emite señal con nuevas ganancias para la GUI → Arduino.

Una instancia por planta (motor / hotend), cadencia distinta:
    - Motor:  cycle=10s,  ventana=100 muestras (10 s @ Ts=0.1)
    - Hotend: cycle=90s,  ventana=6000 muestras (600 s @ Ts=0.1)

Logs CSV por sesión en logs/.
"""

import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal

from sysid import identify_motor, identify_hotend, simulate_arx
from bo_pid import optimize_pid
from drift_detector import DriftDetector


_BASE = Path(__file__).parent
_LOGS = _BASE / "logs"
_LOGS.mkdir(exist_ok=True)


class BOAdaptativoWorker(QThread):
    """
    Worker adaptativo para una planta (motor o hotend).

    Señales:
        nuevos_valores_ready(planta, Kp, Ki, Kd, itae)
        log_event(dict)  — para mostrar en GUI o consola
        drift_detected(planta, residual_pct)
    """

    nuevos_valores_ready = pyqtSignal(str, float, float, float, float)
    log_event = pyqtSignal(dict)
    drift_detected = pyqtSignal(str, float)

    # Configuración por planta
    CONFIGS = {
        'motor': {
            'cycle_s': 10.0,
            'window_size': 100,
            'min_samples': 50,
            'drift_threshold_pct': 15.0,
            'osc_amp_threshold': 2.0,   # σ_y (RPM) que indica ciclo límite
            'n_calls_bo': 10,
            'arx_d': 1,
            'identify_fn': staticmethod(identify_motor),
            'initial_pid': (1.8, 0.9, 0.3),  # Manual año pasado motor (corregido)
            'default_setpoint': 30.0,   # RPM nominal de operación
            'sse_threshold': 1.5,       # |y_mean - sp| > 1.5 RPM dispara
            'sse_settle_std': 0.5,      # σ_y < 0.5 → realmente estable (no transitorio)
            'sse_min_cycles': 2,        # # ciclos consecutivos con SSE antes de forzar BO
        },
        'hotend': {
            'cycle_s': 30.0,          # antes 90 — drift check más frecuente
            'window_size': 1200,      # ventana 120s — captura dinámica térmica
            'min_samples': 600,       # primera id a los 60s
            'drift_threshold_pct': 30.0,   # con predicción a 1 paso el residuo es realista (~22% en estado estable)
            'osc_amp_threshold': 0.8,   # σ_y (°C) que indica ciclo límite
            'n_calls_bo': 10,
            'arx_d': 2,
            'identify_fn': staticmethod(identify_hotend),
            'initial_pid': (35.0, 4.0, 3.0),  # BO Fijo hotend (gp_minimize ITAE+SSE)
            'default_setpoint': 200.0,  # °C nominal de operación
            'sse_threshold': 1.0,       # |y_mean - sp| > 1.0°C dispara
            'sse_settle_std': 0.3,      # σ_y < 0.3°C → estabilizado en mínimo local malo
            'sse_min_cycles': 2,        # 2 ciclos × 30s = 60s con SSE antes de forzar BO
        },
    }

    def __init__(self, planta, initial_model=None, parent=None):
        """
        Args:
            planta: 'motor' o 'hotend'
            initial_model: tupla (num, den) del modelo offline (warm start del modelo).
                           Si es None, el worker espera a juntar datos para identificar.
        """
        super().__init__(parent)

        if planta not in self.CONFIGS:
            raise ValueError(f"planta debe ser 'motor' o 'hotend', no {planta}")

        self.planta = planta
        self.cfg = self.CONFIGS[planta]

        # Modelo actual (G(z) discreto)
        if initial_model is not None:
            self.num, self.den = initial_model
            self._has_model = True
        else:
            self.num, self.den = None, None
            self._has_model = False

        # PID actual (warm start de BO)
        self.Kp, self.Ki, self.Kd = self.cfg['initial_pid']

        # Buffer de datos (la GUI lo llena cada ciclo)
        self.ventana_u = []
        self.ventana_y = []

        # Setpoint actual (la GUI puede actualizarlo vía set_setpoint).
        # Se usa para detectar steady-state error persistente — un mínimo local
        # malo donde el control queda estable pero con offset (caso típico:
        # Ki muy bajo, control no termina de cerrar el error).
        self.sp_target = self.cfg['default_setpoint']
        self._sse_streak = 0   # ciclos consecutivos con SSE alto

        # Cooldown post-BO solo para la rama SSE. Drift y limit-cycle no usan
        # cooldown porque son eventos genuinos (modelo cambió / control oscila)
        # que deben dispararse cuando sea. SSE en cambio puede ser persistente
        # si BO converge a un mínimo local que tampoco cierra el offset → sin
        # cooldown se entraría en un loop SSE→BO→SSE cada 60s.
        self._last_sse_bo_ts = 0.0
        self._sse_cooldown_s = 300.0   # 5 min

        # Detector de drift
        self.detector = DriftDetector(
            threshold_pct=self.cfg['drift_threshold_pct'],
            min_samples=self.cfg['min_samples'],
            d=self.cfg['arx_d'],
        )

        self.running = True
        self._enabled = True

        # Logging
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = _LOGS / f"adaptativo_{planta}_{ts}.csv"
        self._init_log()

    def _init_log(self):
        with open(self.log_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'timestamp', 'planta', 'evento',
                'residual_pct', 'fit_pct',
                'Kp', 'Ki', 'Kd', 'itae',
                'num', 'den',
            ])

    def _log(self, evento, residual=None, fit=None, itae=None):
        with open(self.log_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                datetime.now().isoformat(timespec='seconds'),
                self.planta, evento,
                f"{residual:.3f}" if residual is not None else "",
                f"{fit:.3f}" if fit is not None else "",
                f"{self.Kp:.4f}", f"{self.Ki:.4f}", f"{self.Kd:.4f}",
                f"{itae:.4f}" if itae is not None else "",
                str(list(self.num)) if self.num is not None else "",
                str(list(self.den)) if self.den is not None else "",
            ])

    # ------------------------------------------------------------------
    # API pública desde la GUI
    # ------------------------------------------------------------------

    def actualizar_ventana(self, u_list, y_list):
        """La GUI llama a esto periódicamente con la ventana más reciente."""
        if len(u_list) != len(y_list):
            return
        self.ventana_u = list(u_list[-self.cfg['window_size']:])
        self.ventana_y = list(y_list[-self.cfg['window_size']:])

    def set_pid_actual(self, Kp, Ki, Kd):
        """Si la GUI cambia las ganancias manualmente, actualizamos el warm start."""
        self.Kp, self.Ki, self.Kd = float(Kp), float(Ki), float(Kd)

    def set_setpoint(self, sp):
        """La GUI debe llamar esto cuando cambia el setpoint para que el
        detector de SSE compare contra el target correcto. Si nunca se llama,
        el worker asume el default_setpoint del CONFIGS."""
        self.sp_target = float(sp)
        self._sse_streak = 0   # reset al cambiar referencia

    def enable(self, on):
        self._enabled = bool(on)

    def stop(self):
        self.running = False
        self.wait()

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------

    def run(self):
        self.log_event.emit({
            'planta': self.planta,
            'msg': f"Worker iniciado (cycle={self.cfg['cycle_s']}s)",
        })

        while self.running:
            time.sleep(self.cfg['cycle_s'])

            if not self._enabled:
                continue

            if len(self.ventana_u) < self.cfg['min_samples']:
                self.log_event.emit({
                    'planta': self.planta,
                    'msg': f"Esperando datos: {len(self.ventana_u)}/{self.cfg['min_samples']}",
                })
                continue

            u_std = float(np.std(self.ventana_u))
            y_std = float(np.std(self.ventana_y))
            u_mean = float(np.mean(self.ventana_u))
            y_mean = float(np.mean(self.ventana_y))

            # Detección de Steady-State Error (mínimo local malo).
            # Ocurre cuando el control queda estable pero con offset persistente
            # respecto al setpoint: el modelo de planta es válido, no hay drift
            # ni oscilación, pero el PID no cierra el error (típicamente Ki bajo).
            # Ni la guardia de excitación ni el drift check capturan esto, así
            # que va aquí, ANTES de la guardia, y bypassea la re-id (porque el
            # modelo está bien — solo necesita re-tuneo del PID).
            sse = abs(y_mean - self.sp_target)
            sse_settled = y_std < self.cfg['sse_settle_std']
            if sse > self.cfg['sse_threshold'] and sse_settled:
                self._sse_streak += 1
            else:
                self._sse_streak = 0

            if self._sse_streak >= self.cfg['sse_min_cycles'] and self._has_model:
                cooldown_left = self._sse_cooldown_s - (time.time() - self._last_sse_bo_ts)
                if cooldown_left > 0:
                    self.log_event.emit({
                        'planta': self.planta,
                        'msg': f"🕒 SSE detectado (Δ={sse:.2f}) — BO en cooldown "
                               f"({cooldown_left:.0f}s restantes para próximo intento)",
                    })
                    # NO reseteamos streak — sigue contando para mantener visibilidad
                    continue

                self.log_event.emit({
                    'planta': self.planta,
                    'msg': f"🎯 SSE persistente (Δ={sse:.2f}, σ_y={y_std:.2f}, "
                           f"streak={self._sse_streak}) — re-tuneando PID sin re-id (modelo OK)",
                })
                self._log('sse_detectado', residual=None)
                try:
                    self._run_bo()
                    self._last_sse_bo_ts = time.time()
                except Exception as e:
                    self.log_event.emit({
                        'planta': self.planta,
                        'msg': f"⚠️ BO SSE abortado: {e}",
                    })
                self._sse_streak = 0
                continue

            # Skipear ciclo si no hay excitación suficiente para ARX.
            # La señal correcta es la VARIANZA (σ), no la media: un hotend a
            # setpoint necesita PWM alto y sostenido (compensar pérdidas), eso
            # es operación válida. Solo se skipea cuando PWM/temp están casi
            # constantes (heating saturado, o lazo abierto). NO se resetea el
            # modelo — se conserva el último válido; solo se omite re-identificar.
            if u_std < 8.0 or y_std < 1.0:
                self.log_event.emit({
                    'planta': self.planta,
                    'msg': f"⏸ Sin excitación (μ_u={u_mean:.0f}, σ_u={u_std:.2f}, σ_y={y_std:.2f}) — modelo conservado, sin re-identificar",
                })
                continue

            try:
                self._cycle()
            except Exception as e:
                self.log_event.emit({
                    'planta': self.planta,
                    'msg': f"⚠️ Ciclo abortado: {e}",
                })
                self._log('error_ciclo', residual=None)

    def _cycle(self):
        u = list(self.ventana_u)
        y = list(self.ventana_y)

        # Caso 1: aún no tenemos modelo → identificamos de cero
        if not self._has_model:
            num, den, fit = self.cfg['identify_fn'](u, y)
            if fit < 30.0:
                # Modelo malo → no aceptar, esperar al próximo ciclo con más data
                self.log_event.emit({
                    'planta': self.planta,
                    'msg': f"⏸ Modelo inicial rechazado (FIT={fit:.1f}% < 30%) — esperando más data",
                })
                self._log('identify_inicial_rechazado', fit=fit)
                return
            self.num, self.den = num, den
            self._has_model = True
            self.log_event.emit({
                'planta': self.planta,
                'msg': f"Modelo inicial identificado (FIT={fit:.1f}%)",
            })
            self._log('identify_inicial', fit=fit)
            self._run_bo()
            return

        # Caso 2: ya tenemos modelo → checar drift
        has_drift, residual = self.detector.check(self.num, self.den, u, y)

        self.log_event.emit({
            'planta': self.planta,
            'msg': f"Drift check: residuo={residual:.2f}% "
                   f"(umbral={self.cfg['drift_threshold_pct']}%)",
        })
        self._log('drift_check', residual=residual)

        # Detección de MAL DESEMPEÑO del control: ciclo límite.
        # El drift mira si el MODELO de planta cambió — NO si el control es
        # bueno. Un PID con Ki muy alto oscila sostenidamente alrededor del
        # setpoint aunque el modelo siga siendo perfectamente válido (residuo
        # bajo). Detectamos eso: salida oscilando (σ_y alto) SIN tendencia
        # (media estable entre la 1ª y 2ª mitad de la ventana → no es un
        # transitorio de calentamiento, es un ciclo límite en régimen).
        y_arr = np.array(y)
        n = len(y_arr)
        trend = abs(float(np.mean(y_arr[n // 2:])) - float(np.mean(y_arr[:n // 2])))
        osc_amp = float(np.std(y_arr))
        is_limit_cycle = (trend < 1.0) and (osc_amp > self.cfg['osc_amp_threshold'])

        if not has_drift and not is_limit_cycle:
            return

        # Disparar re-tuneo: por drift de planta O por mal desempeño del control
        if has_drift:
            self.drift_detected.emit(self.planta, residual)
        else:
            self.log_event.emit({
                'planta': self.planta,
                'msg': f"🔄 Ciclo límite detectado (oscilación σ={osc_amp:.2f} "
                       f"sin tendencia) — el modelo es válido pero el control "
                       f"oscila; re-sintonizando",
            })

        num_new, den_new, fit = self.cfg['identify_fn'](u, y)

        if fit < 30.0:
            self.log_event.emit({
                'planta': self.planta,
                'msg': f"⚠️ Re-id rechazada (FIT={fit:.1f}% muy bajo) — reseteo modelo",
            })
            self._log('reid_rechazada', residual=residual, fit=fit)
            # Resetear el modelo cuando FIT se va a negativo
            # En el próximo ciclo se forzará una identificación inicial fresca
            if fit < 0.0:
                self._has_model = False
                self.num, self.den = None, None
            return

        self.num, self.den = num_new, den_new
        self.log_event.emit({
            'planta': self.planta,
            'msg': f"Modelo actualizado (FIT={fit:.1f}%)",
        })
        self._log('reidentificado', residual=residual, fit=fit)

        self._run_bo()

    def _run_bo(self):
        """Corre BO con el modelo actual y warm start desde Kp/Ki/Kd actuales."""
        result = optimize_pid(
            self.num, self.den, Ts=0.1,
            planta=self.planta,
            x0=[self.Kp, self.Ki, self.Kd],
            n_calls=self.cfg['n_calls_bo'],
            verbose=False,
        )

        if not result['success']:
            self.log_event.emit({
                'planta': self.planta,
                'msg': f"⚠️ BO falló: {result.get('error', 'desconocido')}",
            })
            self._log('bo_fallo')
            return

        Kp_new = result['Kp']
        Ki_new = result['Ki']
        Kd_new = result['Kd']
        itae = result['itae']

        # Actualizar estado interno
        self.Kp, self.Ki, self.Kd = Kp_new, Ki_new, Kd_new

        # Emitir a la GUI
        self.nuevos_valores_ready.emit(
            self.planta, Kp_new, Ki_new, Kd_new, itae
        )

        self.log_event.emit({
            'planta': self.planta,
            'msg': f"✅ Nuevas ganancias Kp={Kp_new:.3f}, Ki={Ki_new:.3f}, "
                   f"Kd={Kd_new:.4f}, ITAE={itae:.3f}",
        })
        self._log('bo_exitoso', itae=itae)

        # Limpiar la ventana: los datos que tenemos venían de antes del cambio
        # de PID. Mezclados con los nuevos darían drift falsos o limit-cycle
        # espurio en los próximos ciclos. Esperar a que se reconstruya con
        # datos 100% post-BO antes de volver a evaluar drift.
        self.ventana_u.clear()
        self.ventana_y.clear()
