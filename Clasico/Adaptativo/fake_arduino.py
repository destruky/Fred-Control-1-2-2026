"""
fake_arduino.py — Arduino simulado para modo SIMULACIÓN.

Imita la interfaz de pyserial.Serial: read(), readline(), write().
Internamente corre el lazo cerrado PID + planta motor y hotend, con la
misma lógica que MAIN_F.ino:
    - PWM saturado [0, 255]
    - Zona muerta motor PWM < 30
    - Shutdown hotend si temp > 255 °C
    - Acepta cambios de Kp/Ki/Kd via "write"
    - Devuelve líneas tipo "Motor DC RPM:XX.X" y "Temp:YY.Y" como el real

Permite probar el worker adaptativo sin tener el FrED conectado.

Para inducir drift (probar la respuesta adaptativa), se permite cambiar
los coeficientes de la "planta verdadera" en cualquier momento via
set_drift().
"""

import time
import threading
import queue
import numpy as np
from sysid import simulate_arx


class FakeArduino:
    """
    Reemplazo de serial.Serial para modo simulación.

    Args:
        Ts: tiempo de muestreo (s)
        motor_plant: (num, den) ARX de la planta motor "verdadera"
        hotend_plant: (num, den) ARX de la planta hotend "verdadera"
    """

    def __init__(self,
                 Ts=0.1,
                 motor_plant=None,
                 hotend_plant=None,
                 noise_motor=0.3,
                 noise_hotend=0.5):
        self.Ts = Ts

        # Plantas "verdaderas" — coeficientes realistas extraídos de datos PRBS
        # Motor: ganancia ~0.35 RPM/PWM, τ ≈ 0.2 s.
        # 52 RPM máx a PWM≈150, comportamiento subamortiguado de orden 2.
        self.motor_num = motor_plant[0] if motor_plant else [0.110, -0.063]
        self.motor_den = motor_plant[1] if motor_plant else [1.0, -1.426, 0.561]

        # Hotend: τ ≈ 100 s, retardo d=2.
        # Estado estable: y_ss = (b·u + bias) / (1 - 0.999), con bias=25·0.001
        # Para u=170 → y_ss≈190°C: b ≈ 0.00097
        self.hotend_num = hotend_plant[0] if hotend_plant else [0.00097]
        self.hotend_den = hotend_plant[1] if hotend_plant else [1.0, -0.999]

        self.noise_motor = noise_motor
        self.noise_hotend = noise_hotend

        # Estados
        self.setpoint_motor = 20.0
        self.setpoint_hotend = 25.0
        self.fan_pwm = 0
        # Digits: motor, fan, extrusor, hotend (igual que MAIN_F.ino)
        # "1010" = motor on + heater on (los dos que controla BO adaptativo)
        self.digits = "1001"

        self.Kp_M, self.Ki_M, self.Kd_M = 25.0, 2.5, 1.5
        self.Kp_H, self.Ki_H, self.Kd_H = 1.8, 0.9, 0.3

        # Buffers de simulación
        self._u_motor_hist = [0.0] * 5
        self._y_motor_hist = [0.0] * 5
        self._u_hotend_hist = [0.0] * 5
        self._y_hotend_hist = [25.0] * 5

        # PID estados
        self._eint_M = 0.0
        self._eprev_M = 0.0
        self._dfilt_M = 0.0
        self._eint_H = 0.0
        self._eprev_H = 0.0
        self._dfilt_H = 0.0

        # Cola de líneas a leer
        self._read_buf = queue.Queue()
        self._running = True
        self.is_open = True

        # Hilo de simulación
        self._thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Interfaz pyserial
    # ------------------------------------------------------------------

    def write(self, data):
        """Recibe comandos de la GUI (igual que el Arduino real)."""
        if isinstance(data, bytes):
            data = data.decode('utf-8', errors='ignore')
        cmd = data.strip()

        # Comandos exactos del MAIN_F.ino
        if cmd.startswith("ACTUATE:"):
            try:
                self.digits = cmd.split(":")[1]
            except IndexError:
                pass
        elif cmd.startswith("TEMP:"):
            try:
                self.setpoint_hotend = float(cmd.split(":")[1])
            except (ValueError, IndexError):
                pass
        elif cmd.startswith("DCSPEED:"):
            try:
                self.setpoint_motor = float(cmd.split(":")[1])
            except (ValueError, IndexError):
                pass
        elif cmd.startswith("FANSPEED:"):
            try:
                pct = int(cmd.split(":")[1])
                self.fan_pwm = max(0, min(255, int(pct * 255 / 100)))
            except (ValueError, IndexError):
                pass
        elif cmd.startswith("PIDH:"):
            try:
                vals = cmd.split(":")[1].split(",")
                self.Kp_H, self.Ki_H, self.Kd_H = (float(v) for v in vals[:3])
                self._eint_H = 0.0  # reset al cambiar ganancias
            except (ValueError, IndexError):
                pass
        elif cmd.startswith("PIDM:"):
            try:
                vals = cmd.split(":")[1].split(",")
                self.Kp_M, self.Ki_M, self.Kd_M = (float(v) for v in vals[:3])
                self._eint_M = 0.0
            except (ValueError, IndexError):
                pass
        elif cmd.startswith("SPEED:"):
            pass  # extrusor stepper — no relevante para BO adaptativo

        return len(data)

    def readline(self):
        """Lee una línea como pyserial.readline()."""
        try:
            line = self._read_buf.get(timeout=0.5)
            return line.encode('utf-8')
        except queue.Empty:
            return b""

    def read(self, n=1):
        return self.readline()[:n]

    def close(self):
        self._running = False
        self.is_open = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    @property
    def in_waiting(self):
        return self._read_buf.qsize()

    # ------------------------------------------------------------------
    # Inducir drift desde fuera (para tests / demo)
    # ------------------------------------------------------------------

    def set_motor_drift(self, num, den):
        """Cambia la planta motor real (simula desgaste, fricción nueva)."""
        self.motor_num = list(num)
        self.motor_den = list(den)

    def set_hotend_drift(self, num, den):
        """Cambia la planta hotend real (simula otro filamento, ventilador)."""
        self.hotend_num = list(num)
        self.hotend_den = list(den)

    # ------------------------------------------------------------------
    # Loop interno de simulación
    # ------------------------------------------------------------------

    def _sim_loop(self):
        Tf_M = 5 * self.Ts
        Tf_H = 5 * self.Ts

        while self._running:
            time.sleep(self.Ts)

            motor_on = len(self.digits) >= 1 and self.digits[0] == '1'
            hotend_on = len(self.digits) >= 4 and self.digits[3] == '1'

            # ---- PID Motor ----
            if motor_on:
                y_M = self._y_motor_hist[-1]
                e_M = self.setpoint_motor - y_M
                d_raw = (e_M - self._eprev_M) / self.Ts
                self._dfilt_M = (Tf_M * self._dfilt_M + self.Kd_M * d_raw) / (Tf_M + self.Ts)
                u_raw = self.Kp_M * e_M + self.Ki_M * self._eint_M + self._dfilt_M

                # Zona muerta motor PWM < 30 (igual que MOTORDC.h)
                if u_raw > 255:
                    u_sat = 255
                elif u_raw < 0:
                    u_sat = 0
                elif u_raw > 0 and u_raw < 30:
                    u_sat = 30
                else:
                    u_sat = u_raw

                # Anti-windup
                if u_raw == u_sat:
                    self._eint_M += e_M * self.Ts
                self._eprev_M = e_M

                # Avanzar planta motor (ARX)
                self._u_motor_hist.append(float(u_sat))
                self._u_motor_hist = self._u_motor_hist[-10:]

                y_next_M = self._arx_step(self.motor_num, self.motor_den,
                                          self._u_motor_hist, self._y_motor_hist, d=1)
                y_next_M = max(0.0, y_next_M + np.random.normal(0, self.noise_motor))
                self._y_motor_hist.append(y_next_M)
                self._y_motor_hist = self._y_motor_hist[-10:]
            else:
                u_sat = 0
                y_next_M = 0.0
                self._u_motor_hist.append(0.0)
                self._u_motor_hist = self._u_motor_hist[-10:]
                self._y_motor_hist.append(0.0)
                self._y_motor_hist = self._y_motor_hist[-10:]

            # ---- PID Hotend ----
            if hotend_on:
                y_H = self._y_hotend_hist[-1]
                e_H = self.setpoint_hotend - y_H
                d_raw_H = (e_H - self._eprev_H) / self.Ts
                self._dfilt_H = (Tf_H * self._dfilt_H + self.Kd_H * d_raw_H) / (Tf_H + self.Ts)
                u_raw_H = self.Kp_H * e_H + self.Ki_H * self._eint_H + self._dfilt_H
                u_sat_H = max(0, min(255, u_raw_H))

                if u_raw_H == u_sat_H:
                    self._eint_H += e_H * self.Ts
                self._eprev_H = e_H

                self._u_hotend_hist.append(float(u_sat_H))
                self._u_hotend_hist = self._u_hotend_hist[-10:]

                ambient = 25.0
                # Bias constante para que con u=0 el equilibrio sea ambient.
                # En estado estacionario: A(1) * y_eq = bias  →  y_eq = bias / A(1).
                # A(1) = 1 + sum(den[1:]).  Para y_eq = ambient con u=0:
                #   bias = ambient * A(1) = ambient * (1 + sum(den[1:]))
                A_at_1 = 1.0 + sum(self.hotend_den[1:])
                bias_eq = ambient * A_at_1

                y_next_H = self._arx_step(self.hotend_num, self.hotend_den,
                                          self._u_hotend_hist, self._y_hotend_hist, d=2,
                                          bias=bias_eq)
                y_next_H = max(ambient, y_next_H + np.random.normal(0, self.noise_hotend))

                # Shutdown si temp > 255 (igual que el firmware)
                if y_next_H > 255:
                    y_next_H = 255
                self._y_hotend_hist.append(y_next_H)
                self._y_hotend_hist = self._y_hotend_hist[-10:]
            else:
                u_sat_H = 0
                y_next_H = 25.0
                self._u_hotend_hist.append(0.0)
                self._u_hotend_hist = self._u_hotend_hist[-10:]
                self._y_hotend_hist.append(25.0)
                self._y_hotend_hist = self._y_hotend_hist[-10:]

            # Emitir líneas a Ts (10 Hz) como el firmware adaptativo parchado
            self._read_buf.put(f"Motor DC RPM:{y_next_M:.2f}\n")
            self._read_buf.put(f"PWM Motor:{int(u_sat)}\n")
            self._read_buf.put(f"Temp:{y_next_H:.2f}\n")
            self._read_buf.put(f"PWM Hotend:{int(u_sat_H)}\n")

    @staticmethod
    def _arx_step(num, den, u_hist, y_hist, d=1, bias=0.0):
        """
        Predice y(k+1) dado historial reciente con modelo ARX:
            y(k+1) = -a1·y(k) - ... - ana·y(k-na+1)
                   + b0·u(k+1-d) + b1·u(k-d) + ... + bias

        Convención: u_hist[-1]=u(k), y_hist[-1]=y(k).
        """
        na = len(den) - 1
        nb = len(num)
        a = np.array(den[1:])
        b = np.array(num)

        # Parte autoregresiva
        if na > 0:
            if len(y_hist) < na:
                return 0.0
            y_recent = np.array(y_hist[-na:][::-1])  # [y(k), y(k-1), ...]
            y_part = -np.dot(a, y_recent)
        else:
            y_part = 0.0

        # Parte de entrada: necesitamos u(k+1-d), u(k-d), ..., u(k+1-d-nb+1)
        needed = d + nb - 1
        if len(u_hist) < needed:
            return float(y_part + bias)
        if d == 1:
            u_recent = np.array(u_hist[-nb:][::-1])
        else:
            u_recent = np.array(u_hist[-(d + nb - 1):-(d - 1)][::-1])
        u_part = np.dot(b, u_recent)

        return float(y_part + u_part + bias)


if __name__ == "__main__":
    # Test: simular 5 segundos con setpoint motor=30, hotend=190
    fake = FakeArduino()
    fake.write(b"ACTUATE:1001\n")
    fake.write(b"DCSPEED:30\n")
    fake.write(b"TEMP:190\n")

    print("Simulando 5 s...")
    time.sleep(5.0)
    print(f"Líneas en cola: {fake.in_waiting}")
    for _ in range(10):
        line = fake.readline()
        if line:
            print(f"  {line.decode().strip()}")
    fake.close()
