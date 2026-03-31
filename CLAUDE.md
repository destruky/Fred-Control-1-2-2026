# FrED-TEC — Control del extrusor de filamento 3D
Repo: github.com/destruky/Fred-Control-1-2-2026 | Local: `C:\Users\rquin\OneDrive\Desktop\Fred-Testing\FrED-TEC`
Motor DC (PWM→RPM) + Hotend (PWM→°C). Dos equipos paralelos — **nunca mezclar enfoques**.

## Hardware
Arduino Mega 2560 + RAMPS 1.4. Ts=0.1s, 115200 baud.
Motor: encoder 4704p/rev, zona muerta PWM<30, max ~52 RPM, τ≈0.2s.
Hotend: Steinhart-Hart (A=1.1384e-3, B=2.3245e-4, C=9.489e-8, Raux=460Ω), shutdown >255°C, τ≈100s.
Pinout: `pinMotor=9`, `pinHotend=10`, `pinFan=8`, `termPin=A13`, encoder `C1=18/C2=19`

## Equipos — CRÍTICO: no cruzar enfoques
**Clásico/** G(s) con `tfest()` → PID via Simulink PID Tuner → ML ajusta Kp/Ki/Kd. Multi-exp: `merge()` en MATLAB, nunca concatenar. PROHIBIDO: `ssest`, `n4sid`, espacio de estados, LQR.
**Moderno/** NN PyTorch → Jacobiano → A,B,C,D → LQR en MATLAB. PROHIBIDO: `tfest()`, funciones de transferencia, PID manual.

## Ganancias PID — fuente de verdad: `MAIN_F.ino`
⚠️ Reporte ExpoIngeniería tiene valores **invertidos** — no usar para ganancias.
Motor: Kp=25, Ki=2.5, Kd=1.5 | Hotend: Kp=1.8, Ki=0.9, Kd=0.3

## Red Neuronal Motor (`Moderno/NN/02_motor_sysid_nn.py`)
Feedforward 12→32→32→1, W=5. Input: `[pwm(k-5..k), rpm(k-5..k)]` → `rpm(k+1)`.
Norm: PWM/255, RPM/55. Op point: PWM=75, RPM=28 (eigenvalues estables). Rutas: `Path(__file__).parent`.

## Datos — todos en `Clasico/PRBS_Test/Info/Buena/`
**Motor PRBS:** Motor1=TRAIN, Motor2=VAL, Motor3=TEST. PWM: 30/60/90/120/150, dwell=1.5s, formato `t_ms,pwm,rpm`. `Info/Xs/` tiene RPMs negativos — **no usar**.
**Hotend (escalón):** hotend60/100/180/220.csv. **No usar PRBS** — τ≈100s necesita dwell≥300s, PRBS produce ganancia negativa.

## Estado (2026-03-31)
✅ G(s) motor (~71% FIT), G(s) hotend (~77-88%), NN motor (RMSE=0.54, R²=0.9988), A/B/C/D motor, NN hotend.
⏳ A/B/C/D hotend, Simulink Clásico, LQR motor+hotend, PIDs en FrED físico, LQR en FrED físico, control adaptativo.

## Plan (deadline ~17 abril 2026)
**1–7 abr:** Clásico: Simulink+PID Tuner. Moderno: A/B/C/D hotend + LQR motor+hotend en MATLAB.
**8–14 abr:** Ambos: implementar y validar en FrED físico.
**15–17 abr:** Control adaptativo (Clásico: ML ajusta PID; Moderno: re-linealización online). Comparativa.

## Arduino — errores conocidos
1. `HOTEND.h` declara `prevTime` global → usar `prevTime_sample` en los `.ino`
2. Arduino IDE compila todos los `.ino` de una carpeta → tests en carpeta separada con sus headers
3. Zona muerta motor: PID nunca debe outputar PWM<30
4. Seguridad hotend: shutdown si temp>255°C o `thermistor()==-999`
5. `OG/MAIN_F/` es el original — **no modificar nunca**
