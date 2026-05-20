# Clásico — BO Adaptativo

Módulo adaptativo para el control PID Clásico del Al-FrED0. Detecta drift en
la dinámica de la planta, re-identifica G(z) en línea y resintoniza Kp/Ki/Kd
mediante Optimización Bayesiana con warm start.

Análogo en arquitectura a `Moderno/Adaptativo/` (mismo patrón QThread + GUI),
pero con `tfest()`+BO en vez de NN+LQR.

---

## Estructura

```
Clasico/Adaptativo/
  sysid.py                              Identificación ARX (equivalente a tfest)
  bo_pid.py                             BO con scikit-optimize + warm start
  drift_detector.py                     Detección de drift por residuos (NRMSE)
  fake_arduino.py                       Arduino simulado para modo simulación
  BO_Adaptativo_Worker.py               QThread análogo a Moderno
  AlFrED0_GUI_Adaptativo.py             GUI con toggle simulación / real
  Arduino/MAIN_F_Adaptativo/            Firmware patcheado (10 Hz + PWM en serial)
    MAIN_F_Adaptativo.ino
    MOTORDC.h, HOTEND.h, Pin_map.h
  logs/                                 CSV automático por sesión
```

---

## Instalación

```bash
pip install numpy scipy scikit-optimize PyQt5 pyserial matplotlib
```

---

## Uso rápido

```bash
python AlFrED0_GUI_Adaptativo.py
```

Por defecto arranca en **modo simulación** (FakeArduino).

### Modo simulación (sin FrED conectado)

1. Marca "Modo Simulación".
2. Setpoint: RPM = 30, Temp = 190 → "Aplicar".
3. Espera ~30 s a que se acumule ventana de datos.
4. Marca "Adaptativo Motor" → primer ciclo identifica modelo.
5. Pulsa "Drift Motor" → la planta cambia internamente.
6. Próximo ciclo (10 s) detecta drift, re-identifica, corre BO, manda nuevo PID.
7. Ver consola y plot K(t) para confirmar.

### Modo real (con FrED conectado)

1. Conectar Arduino Mega.
2. Desmarca "Modo Simulación" (auto-conecta vía COM).
3. Igual flujo: setpoint → activar adaptativo → operar normalmente.

---

## Cadencia y ventanas

| Planta | Cycle | Ventana datos | Min muestras | n_calls BO |
|--------|-------|---------------|--------------|------------|
| Motor | 10 s | 100 (10 s) | 50 | 10 (warm) |
| Hotend | 90 s | 6000 (600 s) | 3000 | 10 (warm) |

Umbrales de drift:
- Motor: NRMSE > 15%
- Hotend: NRMSE > 10%

---

## Protocolo serial (idéntico a `MAIN_F.ino`)

GUI → Arduino:
```
SET_RPM:<valor>
SET_TEMP:<valor>
SET_FAN:<valor>
PID_M:Kp,Ki,Kd
PID_H:Kp,Ki,Kd
```

Arduino → GUI (cada Ts = 100 ms):
```
Motor DC RPM:<valor>
PWM Motor:<valor>
Temp:<valor>
PWM Hotend:<valor>
```

> Nota: revisa que el firmware emita las cuatro líneas. Si falta `PWM Motor:`
> o `PWM Hotend:`, el worker no podrá identificar G(z) correctamente.

---

## Logs

Cada sesión genera `logs/adaptativo_<planta>_<timestamp>.csv` con columnas:

```
timestamp, planta, evento, residual_pct, fit_pct,
Kp, Ki, Kd, itae, num, den
```

Eventos posibles: `identify_inicial`, `drift_check`, `reidentificado`,
`reid_rechazada`, `bo_exitoso`, `bo_fallo`, `error_ciclo`.

Estos CSV alimentan **Tabla 7** y **Figura 5** del reporte
(`Reporte_Clasico_FORMATO.md`).

---

## Pipeline

```
Arduino (real o fake)
        │  serial @ Ts=100ms
        ▼
   GUI (buffers u, y)
        │  ventana cada cycle_s
        ▼
   Worker QThread
        ├─ DriftDetector (NRMSE)
        ├─ identify_arx() ── nuevo G(z)
        └─ optimize_pid()  ── nuevo Kp/Ki/Kd
        │
        ▼
   GUI emite por serial: PID_M:... / PID_H:...
        │
        ▼
   Arduino actualiza ganancias
```

---

## Limitaciones conocidas

1. **Identificación en lazo cerrado mal excitado**: si el sistema está en
   régimen permanente con setpoint constante y sin perturbaciones, la
   entrada `u` es prácticamente constante y `identify_arx()` devuelve
   ajustes pobres (FIT bajo o negativo). Mitigaciones:

    - El worker descarta re-identificaciones con FIT < 30%.
    - En la práctica, los cambios naturales de setpoint del operario y las
      perturbaciones del sistema (filamento, ventilador) excitan lo
      suficiente para que la ID funcione en ventanas largas.
    - En modo simulación, usar el botón "Inducir drift" provoca un cambio
      de planta abrupto que sí excita lo necesario.

2. **Modelo offline como warm start**: el worker arranca sin modelo y
   espera datos para identificar uno. Mejor práctica: precargar el modelo
   identificado offline desde `Clasico/PRBS_Test/Info/Buena/motor_Gs.mat` y
   `hotend_Gs.mat`, y solo re-identificar cuando se confirme drift. Esto
   está como TODO opcional: pasar `initial_model=(num, den)` al constructor
   del `BOAdaptativoWorker` para evitar la fase de identificación inicial.

3. **Ciclo BO de hotend**: 90 s entre re-tunings + ~1.5 s de cómputo BO.
   Para ver una respuesta adaptativa visible en demo de 5 minutos, ajustar
   manualmente `cfg['cycle_s']` a 30 s. La calidad de identificación cae
   con ventana más corta.

---

## Decisiones de diseño

- **Stack 100% Python** (no MATLAB Engine) por simetría con Moderno y robustez en demo.
- **Warm start** desde Kp/Ki/Kd actuales → BO converge con ~10 evaluaciones (no 25–60).
- **Re-identificación condicional**: BO solo corre si el drift supera el umbral. Sin drift, las ganancias no cambian → estable, monótono.
- **Hotend aún adaptativo** pero con cadencia mayor (90 s) y ventana de 600 s, justificado en el reporte por τ ≈ 100 s.
- **FakeArduino** permite probar sin hardware. Mismo protocolo serial → la GUI no distingue.

---

## Para llenar el reporte

Una vez corrido el experimento (simulación + hardware), exportar del CSV:

- **Tabla 7 / Figura 5**: comparativa PID estático BO vs Adaptativo
- **Curva K(t)**: del plot inferior derecho de la GUI o del propio CSV
- **Eventos de drift**: contados del CSV → muestran cuántas veces el sistema se readaptó
