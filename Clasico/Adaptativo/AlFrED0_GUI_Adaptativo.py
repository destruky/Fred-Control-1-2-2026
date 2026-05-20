"""
AlFrED0_GUI_Adaptativo.py — GUI extendida para PID Clásico con BO Adaptativo.

Diferencias vs AlFrED0_GUI.py:
    - Toggle "Modo Simulación" / "Modo Real"
    - Toggle "Adaptativo Motor ON/OFF"
    - Toggle "Adaptativo Hotend ON/OFF"
    - Plot de Kp/Ki/Kd vs tiempo (muestra cada actualización del worker)
    - Consola de eventos del worker
    - CSV automático de la sesión

Protocolo serial (idéntico a MAIN_F.ino + MAIN_F_Adaptativo.ino):
    ACTUATE:1001\n        (motor + hotend ON; los demás OFF)
    DCSPEED:<valor>\n     (setpoint motor RPM)
    TEMP:<valor>\n        (setpoint hotend °C)
    FANSPEED:<0-100>\n
    PIDM:Kp,Ki,Kd\n       (ganancias motor)
    PIDH:Kp,Ki,Kd\n       (ganancias hotend)

El firmware MAIN_F_Adaptativo.ino responde a 10 Hz con:
    Motor DC RPM:<valor>
    PWM Motor:<valor>
    Temp:<valor>
    PWM Hotend:<valor>

(El MAIN_F.ino original solo emite Motor DC RPM y Temp a 1 Hz.
 Para BO Adaptativo flashear MAIN_F_Adaptativo.ino.)
"""

import sys
import platform
import time
import csv
import json
from collections import deque
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QGridLayout, QDoubleSpinBox, QCheckBox,
    QPlainTextEdit, QComboBox, QFrame, QFileDialog
)
from PyQt5.QtCore import Qt, QTimer

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from fake_arduino import FakeArduino
from BO_Adaptativo_Worker import BOAdaptativoWorker


# ------------------------------------------------------------------
# Detección de Arduino real
# ------------------------------------------------------------------

def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(s in desc for s in ("arduino", "usb serial", "ch340")):
            return p.device
        if any(s in hwid for s in ("1a86", "2341")):
            return p.device

    if platform.system() == "Windows":
        return "COM7"
    if platform.system() == "Linux":
        return "/dev/ttyACM0"
    if platform.system() == "Darwin":
        return "/dev/tty.usbmodem1101"
    return None


# ------------------------------------------------------------------
# Canvas matplotlib
# ------------------------------------------------------------------

class MultiPlotCanvas(FigureCanvas):
    def __init__(self, n_axes=1, height=2.5, dpi=90):
        fig = Figure(figsize=(6, height * n_axes), dpi=dpi)
        fig.subplots_adjust(left=0.10, right=0.97, top=0.95, bottom=0.10,
                            hspace=0.45)
        self.axes = [fig.add_subplot(n_axes, 1, i + 1) for i in range(n_axes)]
        super().__init__(fig)
        for ax in self.axes:
            ax.grid(True, alpha=0.3)


# ------------------------------------------------------------------
# Ventana principal
# ------------------------------------------------------------------

class AlFrED0Adaptativo(QWidget):
    Ts = 0.1                  # tiempo de muestreo Arduino
    BUFFER_SECONDS = 700      # 700 s de historia para hotend (worker pide 600)
    PLOT_SECONDS = 60         # ventana visible en plot

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Al-FrED0 — PID Clásico con BO Adaptativo")
        self.resize(1400, 900)

        # Conexión (se configura al arrancar workers)
        self.arduino = None
        self.is_simulation = False  # default: hardware real

        # Buffers circulares
        N_BUF = int(self.BUFFER_SECONDS / self.Ts)
        self.t_hist = deque(maxlen=N_BUF)
        self.rpm_hist = deque(maxlen=N_BUF)
        self.pwm_motor_hist = deque(maxlen=N_BUF)
        self.temp_hist = deque(maxlen=N_BUF)
        self.pwm_hotend_hist = deque(maxlen=N_BUF)

        # Historial de ganancias (para plot K(t))
        self.t_pid = []
        self.Kp_M_hist, self.Ki_M_hist, self.Kd_M_hist = [], [], []
        self.Kp_H_hist, self.Ki_H_hist, self.Kd_H_hist = [], [], []

        self.t0 = time.time()

        # Workers
        self.worker_motor = None
        self.worker_hotend = None

        self._build_ui()
        self._connect_arduino()

        # Timer de sampling (lee Arduino cada Ts)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(self.Ts * 1000))

        # Timer de actualización de plots (más lento para no saturar)
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_timer.start(500)

    # --------------------------------------------------------------
    # UI
    # --------------------------------------------------------------

    def _build_ui(self):
        root = QHBoxLayout(self)

        # ---- COLUMNA IZQUIERDA: controles ----
        left = QVBoxLayout()
        root.addLayout(left, 1)

        # Modo
        gb_modo = QGroupBox("Modo de operación")
        ly_modo = QVBoxLayout(gb_modo)
        self.cb_sim = QCheckBox("Modo Simulación (FakeArduino)")
        self.cb_sim.setChecked(False)
        self.cb_sim.toggled.connect(self._on_mode_toggle)
        ly_modo.addWidget(self.cb_sim)
        self.lbl_conn = QLabel("Estado: —")
        ly_modo.addWidget(self.lbl_conn)
        left.addWidget(gb_modo)

        # Activación de subsistemas (ACTUATE)
        gb_act = QGroupBox("Activación")
        ly_act = QHBoxLayout(gb_act)
        self.btn_start = QPushButton("▶ Iniciar control (ACTUATE:1001)")
        self.btn_start.clicked.connect(self._on_start_control)
        ly_act.addWidget(self.btn_start)
        self.btn_stop = QPushButton("■ Detener (ACTUATE:0000)")
        self.btn_stop.clicked.connect(self._on_stop_control)
        ly_act.addWidget(self.btn_stop)
        self.btn_export = QPushButton("📁 Exportar CSV")
        self.btn_export.setStyleSheet(
            "QPushButton { padding: 6px; background-color: #2980b9; color: white; "
            "font-weight: bold; border-radius: 4px; }"
            "QPushButton:hover { background-color: #3498db; }"
        )
        self.btn_export.clicked.connect(self._export_csv)
        ly_act.addWidget(self.btn_export)
        left.addWidget(gb_act)

        # Setpoints
        gb_sp = QGroupBox("Setpoints")
        ly_sp = QGridLayout(gb_sp)
        ly_sp.addWidget(QLabel("RPM Motor:"), 0, 0)
        self.sp_motor = QDoubleSpinBox()
        self.sp_motor.setRange(0, 55)
        self.sp_motor.setValue(30)
        self.sp_motor.setDecimals(1)
        ly_sp.addWidget(self.sp_motor, 0, 1)
        btn_sp_m = QPushButton("Aplicar")
        btn_sp_m.clicked.connect(self._send_sp_motor)
        ly_sp.addWidget(btn_sp_m, 0, 2)

        ly_sp.addWidget(QLabel("Temp Hotend (°C):"), 1, 0)
        self.sp_hotend = QDoubleSpinBox()
        self.sp_hotend.setRange(0, 250)
        self.sp_hotend.setValue(200)
        self.sp_hotend.setDecimals(1)
        ly_sp.addWidget(self.sp_hotend, 1, 1)
        btn_sp_h = QPushButton("Aplicar")
        btn_sp_h.clicked.connect(self._send_sp_hotend)
        ly_sp.addWidget(btn_sp_h, 1, 2)

        left.addWidget(gb_sp)

        # PID actuales
        gb_pid = QGroupBox("PID actuales (auto-actualizado por adaptativo)")
        ly_pid = QGridLayout(gb_pid)
        ly_pid.addWidget(QLabel("Motor:"), 0, 0)
        self.lbl_pid_M = QLabel("Kp=1.800  Ki=0.900  Kd=0.300")
        ly_pid.addWidget(self.lbl_pid_M, 0, 1)
        ly_pid.addWidget(QLabel("Hotend:"), 1, 0)
        self.lbl_pid_H = QLabel("Kp=35.000  Ki=4.000  Kd=3.000")
        ly_pid.addWidget(self.lbl_pid_H, 1, 1)
        left.addWidget(gb_pid)

        # Adaptativo toggles
        gb_ad = QGroupBox("Módulo Adaptativo (BO)")
        ly_ad = QVBoxLayout(gb_ad)
        self.cb_ad_motor = QCheckBox("Adaptativo Motor (ciclo 10 s)")
        self.cb_ad_motor.toggled.connect(self._toggle_motor_worker)
        ly_ad.addWidget(self.cb_ad_motor)
        self.cb_ad_hotend = QCheckBox("Adaptativo Hotend (ciclo 30 s)")
        self.cb_ad_hotend.toggled.connect(self._toggle_hotend_worker)
        ly_ad.addWidget(self.cb_ad_hotend)
        left.addWidget(gb_ad)

        # Inducir drift (solo simulación)
        gb_drift = QGroupBox("Inducir drift (simulación)")
        ly_drift = QVBoxLayout(gb_drift)
        self.btn_drift_motor = QPushButton("Drift Motor (cambiar planta)")
        self.btn_drift_motor.clicked.connect(self._induce_drift_motor)
        ly_drift.addWidget(self.btn_drift_motor)
        self.btn_drift_hotend = QPushButton("Drift Hotend (cambiar planta)")
        self.btn_drift_hotend.clicked.connect(self._induce_drift_hotend)
        ly_drift.addWidget(self.btn_drift_hotend)
        left.addWidget(gb_drift)

        # Consola
        gb_log = QGroupBox("Eventos del módulo adaptativo")
        ly_log = QVBoxLayout(gb_log)
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(500)
        ly_log.addWidget(self.console)
        left.addWidget(gb_log, 2)

        # ---- COLUMNA DERECHA: plots ----
        right = QVBoxLayout()
        root.addLayout(right, 3)

        self.plot_proc = MultiPlotCanvas(n_axes=2, height=2.5)
        right.addWidget(self.plot_proc)

        self.plot_pid = MultiPlotCanvas(n_axes=2, height=2.5)
        right.addWidget(self.plot_pid)

    # --------------------------------------------------------------
    # Conexión Arduino / Fake
    # --------------------------------------------------------------

    def _connect_arduino(self):
        if self.arduino is not None:
            try:
                self.arduino.close()
            except Exception:
                pass
            self.arduino = None

        if self.is_simulation:
            self.arduino = FakeArduino(Ts=self.Ts)
            self.lbl_conn.setText("Estado: SIMULACIÓN (FakeArduino)")
        else:
            puerto = encontrar_puerto_arduino()
            if puerto is None:
                self.lbl_conn.setText("Estado: ❌ No se encontró Arduino")
                return
            try:
                self.arduino = serial.Serial(puerto, 115200, timeout=0.05)
                self.lbl_conn.setText(f"Estado: ✅ Real ({puerto})")
            except Exception as e:
                self.lbl_conn.setText(f"Estado: ❌ Error: {e}")

    def _on_mode_toggle(self, checked):
        # Apagar workers antes de cambiar
        self._stop_workers()
        self.cb_ad_motor.setChecked(False)
        self.cb_ad_hotend.setChecked(False)
        self.is_simulation = checked
        self._connect_arduino()

    # --------------------------------------------------------------
    # Envío al Arduino
    # --------------------------------------------------------------

    def _send(self, line):
        if self.arduino is None:
            return
        try:
            if not line.endswith("\n"):
                line += "\n"
            self.arduino.write(line.encode())
        except Exception as e:
            self._console_log(f"Error serial: {e}")

    def _send_sp_motor(self):
        self._send(f"DCSPEED:{self.sp_motor.value():.2f}")

    def _send_sp_hotend(self):
        self._send(f"TEMP:{self.sp_hotend.value():.2f}")

    def _send_pid_motor(self, Kp, Ki, Kd):
        self._send(f"PIDM:{Kp:.4f},{Ki:.4f},{Kd:.4f}")

    def _send_pid_hotend(self, Kp, Ki, Kd):
        self._send(f"PIDH:{Kp:.4f},{Ki:.4f},{Kd:.4f}")

    def _send_actuate_motor_hotend(self):
        """Activa motor + hotend en el firmware (digits[0]=motor, digits[3]=hotend)."""
        self._send("ACTUATE:1001")

    def _on_start_control(self):
        """Manda ACTUATE:1001 + setpoints + PIDs iniciales (warm start)."""
        self._send("ACTUATE:1001")
        self._send_sp_motor()
        self._send_sp_hotend()
        # Enviar PIDs iniciales (warm start del BO Fijo) para no usar defaults del Arduino
        self._send_pid_motor(1.8, 0.9, 0.3)
        self._send_pid_hotend(35.0, 4.0, 3.0)
        self._console_log("▶ Control iniciado (motor + hotend ON, PIDs warm-start enviados)")

    def _on_stop_control(self):
        """Apaga todos los actuadores."""
        self._send("ACTUATE:0000")
        self._console_log("■ Control detenido")

    # --------------------------------------------------------------
    # Tick principal — leer Arduino
    # --------------------------------------------------------------

    def _tick(self):
        if self.arduino is None:
            return

        max_iter = 50
        while max_iter > 0:
            try:
                line = self.arduino.readline()
            except Exception:
                break
            if not line:
                break
            try:
                txt = line.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue
            self._parse_line(txt)
            max_iter -= 1

        # Cada tick, añadir muestra al historial usando los últimos valores
        # parseados (si los hay)
        if self._last_rpm is not None and self._last_pwm_M is not None:
            t_now = time.time() - self.t0
            self.t_hist.append(t_now)
            self.rpm_hist.append(self._last_rpm)
            self.pwm_motor_hist.append(self._last_pwm_M)
            self.temp_hist.append(self._last_temp if self._last_temp is not None else float('nan'))
            self.pwm_hotend_hist.append(self._last_pwm_H if self._last_pwm_H is not None else float('nan'))

            # Pasar ventana a workers
            if self.worker_motor is not None:
                self.worker_motor.actualizar_ventana(
                    list(self.pwm_motor_hist), list(self.rpm_hist)
                )
            if self.worker_hotend is not None:
                self.worker_hotend.actualizar_ventana(
                    list(self.pwm_hotend_hist), list(self.temp_hist)
                )

    _last_rpm = None
    _last_pwm_M = None
    _last_temp = None
    _last_pwm_H = None

    def _parse_line(self, txt):
        if "RPM" in txt and ":" in txt:
            try:
                self._last_rpm = float(txt.split(":")[-1])
            except ValueError:
                pass
        elif txt.startswith("PWM Motor:"):
            try:
                self._last_pwm_M = float(txt.split(":")[-1])
            except ValueError:
                pass
        elif txt.startswith("Temp:"):
            try:
                self._last_temp = float(txt.split(":")[-1])
            except ValueError:
                pass
        elif txt.startswith("PWM Hotend:"):
            try:
                self._last_pwm_H = float(txt.split(":")[-1])
            except ValueError:
                pass

    # --------------------------------------------------------------
    # Workers adaptativos
    # --------------------------------------------------------------

    def _toggle_motor_worker(self, checked):
        if checked:
            if self.worker_motor is None:
                self.worker_motor = BOAdaptativoWorker('motor')
                self.worker_motor.nuevos_valores_ready.connect(self._on_new_pid)
                self.worker_motor.log_event.connect(self._on_worker_log)
                self.worker_motor.drift_detected.connect(self._on_drift)
                self.worker_motor.start()
                self._console_log("✅ Worker MOTOR iniciado")
            else:
                self.worker_motor.enable(True)
        else:
            if self.worker_motor is not None:
                self.worker_motor.enable(False)
                self._console_log("⏸ Worker MOTOR pausado")

    def _cargar_modelo_hotend_offline(self):
        """
        Carga el modelo G(z) del hotend identificado offline (BO Fijo) desde
        fred_bo_hotend_results.json. Se pasa al worker como initial_model para
        que arranque con un modelo estable y válido sin depender de la
        identificación online (que falla durante calentamiento saturado).
        """
        json_path = Path(__file__).parent.parent / "MATLAB" / "fred_bo_hotend_results.json"
        try:
            with open(json_path) as f:
                d = json.load(f)
            num, den = d['num'], d['den']
            self._console_log(f"📂 Modelo hotend offline cargado: num={num}, den={den}")
            return (num, den)
        except Exception as e:
            self._console_log(f"⚠️ No se pudo cargar modelo offline ({e}) — worker identificará online")
            return None

    def _toggle_hotend_worker(self, checked):
        if checked:
            if self.worker_hotend is None:
                modelo = self._cargar_modelo_hotend_offline()
                self.worker_hotend = BOAdaptativoWorker('hotend', initial_model=modelo)
                self.worker_hotend.nuevos_valores_ready.connect(self._on_new_pid)
                self.worker_hotend.log_event.connect(self._on_worker_log)
                self.worker_hotend.drift_detected.connect(self._on_drift)
                self.worker_hotend.start()
                self._console_log("✅ Worker HOTEND iniciado")
            else:
                self.worker_hotend.enable(True)
        else:
            if self.worker_hotend is not None:
                self.worker_hotend.enable(False)
                self._console_log("⏸ Worker HOTEND pausado")

    def _stop_workers(self):
        for w in (self.worker_motor, self.worker_hotend):
            if w is not None:
                w.stop()
        self.worker_motor = None
        self.worker_hotend = None

    # --------------------------------------------------------------
    # Slots de los workers
    # --------------------------------------------------------------

    def _on_new_pid(self, planta, Kp, Ki, Kd, itae):
        t_now = time.time() - self.t0
        if planta == 'motor':
            self._send_pid_motor(Kp, Ki, Kd)
            self.lbl_pid_M.setText(f"Kp={Kp:.3f}  Ki={Ki:.3f}  Kd={Kd:.3f}")
            self.t_pid.append(t_now)
            self.Kp_M_hist.append(Kp)
            self.Ki_M_hist.append(Ki)
            self.Kd_M_hist.append(Kd)
        elif planta == 'hotend':
            self._send_pid_hotend(Kp, Ki, Kd)
            self.lbl_pid_H.setText(f"Kp={Kp:.3f}  Ki={Ki:.3f}  Kd={Kd:.3f}")
            self.t_pid.append(t_now)
            self.Kp_H_hist.append(Kp)
            self.Ki_H_hist.append(Ki)
            self.Kd_H_hist.append(Kd)

        self._console_log(f"🔄 [{planta.upper()}] PID actualizado → "
                          f"Kp={Kp:.3f} Ki={Ki:.3f} Kd={Kd:.3f}  ITAE={itae:.2f}")

    def _on_worker_log(self, payload):
        self._console_log(f"[{payload['planta']}] {payload['msg']}")

    def _on_drift(self, planta, residual):
        self._console_log(f"⚠️ DRIFT [{planta.upper()}] residuo={residual:.1f}%")

    # --------------------------------------------------------------
    # Drift inducido (simulación)
    # --------------------------------------------------------------

    def _induce_drift_motor(self):
        if not self.is_simulation or not isinstance(self.arduino, FakeArduino):
            self._console_log("Drift solo disponible en modo simulación")
            return
        # Cambiar planta motor a una con dinámica diferente
        self.arduino.set_motor_drift([0.0, 0.30, 0.10], [1.0, -1.40, 0.55])
        self._console_log("💥 Drift inducido en MOTOR — esperar siguiente ciclo")

    def _induce_drift_hotend(self):
        if not self.is_simulation or not isinstance(self.arduino, FakeArduino):
            self._console_log("Drift solo disponible en modo simulación")
            return
        self.arduino.set_hotend_drift([0.0, 0.0, 0.020], [1.0, -0.996])
        self._console_log("💥 Drift inducido en HOTEND — esperar siguiente ciclo")

    # --------------------------------------------------------------
    # Plots
    # --------------------------------------------------------------

    def _update_plots(self):
        if len(self.t_hist) == 0:
            return

        t = np.array(self.t_hist)
        rpm = np.array(self.rpm_hist)
        temp = np.array(self.temp_hist)

        # Ventana visible
        t_max = t[-1]
        mask = t >= (t_max - self.PLOT_SECONDS)

        # Plot proceso
        ax0, ax1 = self.plot_proc.axes
        ax0.cla()
        ax0.plot(t[mask], rpm[mask], 'b-', linewidth=1.4)
        ax0.axhline(self.sp_motor.value(), color='k', linestyle='--', linewidth=0.8)
        ax0.set_ylabel("RPM Motor")
        ax0.grid(True, alpha=0.3)

        ax1.cla()
        ax1.plot(t[mask], temp[mask], 'r-', linewidth=1.4)
        ax1.axhline(self.sp_hotend.value(), color='k', linestyle='--', linewidth=0.8)
        ax1.set_ylabel("Temp Hotend (°C)")
        ax1.set_xlabel("Tiempo (s)")
        ax1.grid(True, alpha=0.3)

        self.plot_proc.draw()

        # Plot PID
        axp0, axp1 = self.plot_pid.axes
        axp0.cla()
        if self.Kp_M_hist:
            axp0.step(self.t_pid[:len(self.Kp_M_hist)], self.Kp_M_hist, 'b-o',
                      label='Kp', where='post', markersize=3)
            axp0.step(self.t_pid[:len(self.Ki_M_hist)], self.Ki_M_hist, 'g-s',
                      label='Ki', where='post', markersize=3)
            axp0.step(self.t_pid[:len(self.Kd_M_hist)], self.Kd_M_hist, 'r-^',
                      label='Kd', where='post', markersize=3)
            axp0.legend(loc='upper right', fontsize=8)
        axp0.set_ylabel("PID Motor")
        axp0.grid(True, alpha=0.3)

        axp1.cla()
        if self.Kp_H_hist:
            axp1.step(self.t_pid[-len(self.Kp_H_hist):], self.Kp_H_hist, 'b-o',
                      label='Kp', where='post', markersize=3)
            axp1.step(self.t_pid[-len(self.Ki_H_hist):], self.Ki_H_hist, 'g-s',
                      label='Ki', where='post', markersize=3)
            axp1.step(self.t_pid[-len(self.Kd_H_hist):], self.Kd_H_hist, 'r-^',
                      label='Kd', where='post', markersize=3)
            axp1.legend(loc='upper right', fontsize=8)
        axp1.set_ylabel("PID Hotend")
        axp1.set_xlabel("Tiempo (s)")
        axp1.grid(True, alpha=0.3)

        self.plot_pid.draw()

    # --------------------------------------------------------------
    # Helpers
    # --------------------------------------------------------------

    def _console_log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.console.appendPlainText(f"[{ts}] {msg}")

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Guardar CSV", "hw_adapt_session.csv", "CSV Files (*.csv)"
        )
        if not path:
            return
        try:
            n_tel = min(
                len(self.t_hist), len(self.rpm_hist), len(self.pwm_motor_hist),
                len(self.temp_hist), len(self.pwm_hotend_hist),
            )
            n_pid = min(
                len(self.t_pid),
                len(self.Kp_M_hist), len(self.Ki_M_hist), len(self.Kd_M_hist),
                len(self.Kp_H_hist), len(self.Ki_H_hist), len(self.Kd_H_hist),
            )
            n = max(n_tel, n_pid)
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow([
                    'Time_s', 'RPM_Motor', 'PWM_Motor',
                    'Temp_Hotend', 'PWM_Hotend',
                    't_pid', 'Kp_Motor', 'Ki_Motor', 'Kd_Motor',
                    'Kp_Hotend', 'Ki_Hotend', 'Kd_Hotend',
                ])
                for i in range(n):
                    row = [
                        self.t_hist[i] if i < n_tel else '',
                        self.rpm_hist[i] if i < n_tel else '',
                        self.pwm_motor_hist[i] if i < n_tel else '',
                        self.temp_hist[i] if i < n_tel else '',
                        self.pwm_hotend_hist[i] if i < n_tel else '',
                        self.t_pid[i] if i < n_pid else '',
                        self.Kp_M_hist[i] if i < n_pid else '',
                        self.Ki_M_hist[i] if i < n_pid else '',
                        self.Kd_M_hist[i] if i < n_pid else '',
                        self.Kp_H_hist[i] if i < n_pid else '',
                        self.Ki_H_hist[i] if i < n_pid else '',
                        self.Kd_H_hist[i] if i < n_pid else '',
                    ]
                    w.writerow(row)
            self._console_log(f"CSV guardado: {path} ({n} filas)")
        except Exception as e:
            self._console_log(f"Error al guardar CSV: {e}")

    def closeEvent(self, ev):
        self._stop_workers()
        if self.arduino is not None:
            try:
                self.arduino.close()
            except Exception:
                pass
        super().closeEvent(ev)


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = AlFrED0Adaptativo()
    win.show()
    sys.exit(app.exec_())

