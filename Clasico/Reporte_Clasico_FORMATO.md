Instituto Tecnológico y de Estudios Superiores de Monterrey — Campus Monterrey

Control PID con Optimización Bayesiana y Ajuste Adaptativo para el extrusor Al-FrED0

Docente: Dr. Erick Guadalupe Ramírez Cedillo — erickramce@tec.mx

Equipo:
Rodrigo Quintero Casso — A01199249
Sergio René Castillo Cantú — A01723797
Mariana Ameyali Aguilar González — A00844156
Hans Enrique Velarde Barrón — A01286990
Adrián Oswaldo Salazar González — A00838435

[FECHA] — Monterrey, Nuevo León

────────────────────────────────────────

RESUMEN

[Problema → solución BO → resultados numéricos (ITAE motor y hotend, mejora vs baseline) → validación Simulink + hardware]

────────────────────────────────────────

1. INTRODUCCIÓN

[Por qué el FrED necesita control preciso. Por qué PID manual es insuficiente. Presentar el pipeline propuesto.]

2. CONTEXTO GENERAL

[Al-FrED0: extrusor de filamento, dos variables a controlar. Diferencia de dinámicas: τ_motor ≈ 0.2 s vs τ_hotend ≈ 100 s.]

3. DELIMITACIÓN DEL OBJETO DE ESTUDIO

Este proyecto controla dos subsistemas del Al-FrED0:
- Motor DC de embobinado (PWM → RPM)
- Hotend (PWM → °C)

Hardware fijo: Arduino Mega 2560 + RAMPS 1.4. Sin modificaciones mecánicas.

4. PLANTEAMIENTO DEL PROBLEMA

[Sintonización manual no es reproducible ni óptima. Consecuencias: overshoot térmico, velocidad inestable, filamento irregular. Se necesita un método sistemático que minimice ITAE automáticamente.]

5. JUSTIFICACIÓN

[Por qué BO sobre otras alternativas (Ziegler-Nichols, PSO, GA). Por qué ITAE como métrica. Relevancia en manufactura aditiva.]

────────────────────────────────────────

6. MARCO TEÓRICO

6.1 Controlador PID Digital
[Ecuación discreta. Rol de Kp, Ki, Kd. Implementación Arduino: saturación PWM 0–255, zona muerta < 30.]

6.2 Identificación de Sistemas
[PRBS para motor (τ rápida, excita múltiples frecuencias). Escalón para hotend (τ ≈ 100 s, PRBS inviable). tfest() en MATLAB. Métrica FIT%.]

6.3 Criterio ITAE
ITAE = Σ k · |e(k)| · Ts
[Por qué ITAE sobre ISE/IAE para este sistema.]

6.4 Optimización Bayesiana
[Proceso gaussiano sobre el espacio [Kp, Ki, Kd]. Función de adquisición: exploración vs explotación. Converge con pocas evaluaciones. bayesopt() en MATLAB.]

Figura A. Diagrama de bloques del sistema de control completo.
[DIAGRAMA: datos experimentales → tfest() → G(s) → BO (bayesopt) → Kp/Ki/Kd óptimos → PID en lazo cerrado → planta → retroalimentación → módulo adaptativo]

6.5 PID Adaptativo
[G(s) cambia con el punto de operación. El módulo re-ejecuta BO con datos recientes para actualizar ganancias en línea.]

────────────────────────────────────────

7. OBJETIVO GENERAL

Sintonizar automáticamente controladores PID para el motor DC y el hotend del Al-FrED0 mediante Optimización Bayesiana, e integrar un módulo adaptativo que ajuste las ganancias en tiempo real, minimizando ITAE frente al PID sintonizado manualmente.

8. OBJETIVOS ESPECÍFICOS

- Identificar G(s) del motor (PRBS) y hotend (escalón) con tfest(), FIT mínimo [XX]%.
- Diseñar el algoritmo BO en MATLAB que minimice ITAE para ambas plantas.
- Comparar BO vs baseline pidtune() en Simulink: ITAE, tiempo de establecimiento, sobretiro.
- Implementar los PID optimizados en Arduino Mega y validar en el Al-FrED0 físico.
- Integrar módulo adaptativo que resintonice ganancias ante cambios de operación.

9. HIPÓTESIS

Si se sintoniza el PID del motor DC y hotend mediante Optimización Bayesiana minimizando ITAE, entonces se reducirá significativamente el error acumulado vs pidtune(), logrando establecimiento menor a [XX] s en el motor y overshoot menor a [XX]% en el hotend.

────────────────────────────────────────

10. METODOLOGÍA

10.1 Recolección de Datos

Motor DC — señal PRBS, niveles PWM {30, 60, 90, 120, 150}, dwell = 1.5 s:
- PRBS_Motor1.csv (entrenamiento)
- PRBS_Motor2.csv (validación)
- PRBS_Motor3.csv (prueba)
Combinados con merge() en MATLAB.

Hotend — escalones PWM {60, 100, 180, 220}, duración ≥ 5τ por nivel:
- hotend60/100/180/220.csv

Figura B. Señal PRBS aplicada al motor DC y respuesta de RPM medida.
[GRÁFICA: tiempo vs PWM (arriba) y tiempo vs RPM (abajo)]

Figura C. Escalones de PWM aplicados al hotend y respuesta de temperatura.
[GRÁFICA: tiempo vs PWM (arriba) y tiempo vs °C (abajo)]

10.2 Identificación de G(s)

Motor DC:    G(s) = [ECUACIÓN] — FIT: [XX]%
Hotend:      G(s) = [ECUACIÓN] — FIT: [XX]%

Scripts: [NOMBRE SCRIPT motor] y [NOMBRE SCRIPT hotend]

Figura D. Ajuste del modelo G(s) vs datos reales — Motor DC.
[GRÁFICA: salida medida vs salida simulada por el modelo, con FIT%]

Figura E. Ajuste del modelo G(s) vs datos reales — Hotend.
[GRÁFICA: salida medida vs salida simulada por el modelo, con FIT%]

10.3 Optimización Bayesiana

Scripts: motor_bayes_opt_2.m | fred_clasico_bo_hotend.m
Función objetivo: ITAE simulado en tiempo discreto (Ts = 0.1 s)

Rangos de búsqueda:

| Parámetro | Motor | Hotend |
|-----------|-------|--------|
| Kp | [XX, XX] | [XX, XX] |
| Ki | [XX, XX] | [XX, XX] |
| Kd | [XX, XX] | [XX, XX] |

Evaluaciones: 25 (motor) | 60 (hotend)

10.4 Validación en Simulink

Modelos: motor_Gs.mat | hotend_Gs.mat
Comparativa: pidtune() baseline vs BO optimizado
- Motor: escalón [XX] RPM, simulación [XX] s
- Hotend: escalón [XX] °C, simulación 800–1000 s

10.5 Implementación en Arduino

Firmware: MAIN_F.ino — Arduino Mega 2560 + RAMPS 1.4
- Motor: pin 9, encoder pines 18/19, zona muerta PWM < 30
- Hotend: pin 10, termistor A13, shutdown > 255 °C
- Ts = 100 ms

Figura F. Estructura de archivos del firmware.
[DIAGRAMA: MAIN_F.ino → HOTEND.h / MOTORDC.h / PIN_MAP.h, con descripción breve de cada archivo]

10.6 PID Adaptativo

[Describir: frecuencia de resintonización, datos que usa, cómo actualiza ganancias en el Arduino.]

────────────────────────────────────────

11. TÉCNICAS Y HERRAMIENTAS

| Herramienta | Uso |
|-------------|-----|
| MATLAB tfest() | Identificación de G(s) |
| MATLAB bayesopt() | Optimización de Kp, Ki, Kd |
| MATLAB merge() | Combinar experimentos PRBS |
| MATLAB pidtune() | Baseline de comparación |
| Simulink (bloque PID) | Validación en simulación |
| Arduino IDE (C++) | Firmware del controlador |
| Python + GUI serial | Monitoreo y envío de setpoints |

12. INFRAESTRUCTURA Y RECURSOS

Hardware:
| Componente | Descripción |
|------------|-------------|
| Arduino Mega 2560 | Microcontrolador |
| RAMPS 1.4 | Shield de potencia |
| Motor DC + encoder | 4704 pulsos/rev, ~52 RPM máx |
| Hotend Artillery | Calentador + termistor Steinhart-Hart |
| Fuente 12V / 20A | Alimentación |

Software: MATLAB R[XXXX] | Arduino IDE | Python 3.X

────────────────────────────────────────

13. RESULTADOS

Tabla 1. Funciones de transferencia identificadas.

| Planta | G(s) | FIT (%) |
|--------|------|---------|
| Motor DC | [ECUACIÓN] | [XX]% |
| Hotend | [ECUACIÓN] | [XX]% |


Tabla 2. Ganancias PID — baseline vs BO optimizado.

| Planta | Fuente | Kp | Ki | Kd | ITAE |
|--------|--------|----|----|----|------|
| Motor DC | pidtune (baseline) | [XX] | [XX] | [XX] | [XX] |
| Motor DC | BO optimizado | [XX] | [XX] | [XX] | [XX] |
| Hotend | pidtune (baseline) | [XX] | [XX] | [XX] | [XX] |
| Hotend | BO optimizado | [XX] | [XX] | [XX] | [XX] |

Nota: ITAE hotend es alto en valor absoluto por duración de simulación (800 s) y τ ≈ 100 s. La mejora se mide de forma relativa al baseline.


Tabla 3. Desempeño en Simulink — Motor DC.

| Métrica | Baseline | BO | Mejora |
|---------|----------|----|--------|
| ITAE | [XX] | [XX] | [XX]% |
| T. establecimiento (s) | [XX] | [XX] | — |
| Sobretiro (%) | [XX] | [XX] | — |
| Error estacionario | [XX] | [XX] | — |

Figura 1. Respuesta escalón Simulink — Motor DC.
[GRÁFICA]


Tabla 4. Desempeño en Simulink — Hotend.

| Métrica | Baseline | BO | Mejora |
|---------|----------|----|--------|
| ITAE | [XX] | [XX] | [XX]% |
| T. establecimiento (s) | [XX] | [XX] | — |
| Sobretiro (%) | [XX] | [XX] | — |
| Error estacionario | [XX] | [XX] | — |

Figura 2. Respuesta escalón Simulink — Hotend.
[GRÁFICA]


Tabla 5. Desempeño en hardware — Motor DC.

| Métrica | Valor |
|---------|-------|
| Setpoint | [XX] RPM |
| T. establecimiento | [XX] s |
| Sobretiro | [XX]% |
| Error estacionario | [XX] RPM |

Tabla 6. Desempeño en hardware — Hotend.

| Métrica | Valor |
|---------|-------|
| Setpoint | [XX] °C |
| T. establecimiento | [XX] s |
| Sobretiro | [XX]% |
| Error estacionario | [XX] °C |

Figura 3. Respuesta motor DC en Al-FrED0 físico.
[GRÁFICA]

Figura 4. Respuesta hotend en Al-FrED0 físico.
[GRÁFICA]


Tabla 7. PID estático BO vs PID Adaptativo (hardware).

| Métrica | PID BO estático | PID Adaptativo |
|---------|----------------|----------------|
| ITAE Motor | [XX] | [XX] |
| ITAE Hotend | [XX] | [XX] |
| Ante cambio de setpoint | [XX] | [XX] |
| Ante perturbación | [XX] | [XX] |

Figura 5. PID estático vs adaptativo — comparativa.
[GRÁFICA]

────────────────────────────────────────

14. CONCLUSIONES

[1 — ¿Se validó la hipótesis? ITAE logrado vs baseline, cifras concretas.]
[2 — Diferencia entre plantas: dinámicas opuestas, estrategias distintas.]
[3 — Limitaciones: Ki al límite del rango en motor, zona muerta PWM, ITAE absoluto hotend.]
[4 — Trabajo futuro: ampliar rangos BO, más evaluaciones, adaptativo en más puntos de operación.]

────────────────────────────────────────

15. REFERENCIAS

[APA — incluir:]
- Åström & Hägglund (2009). PID Controllers: Theory, Design and Tuning.
- Mockus (1989). Bayesian Approach to Global Optimization.
- MATLAB Documentation — bayesopt(), tfest(), pidtune().
- [Fuente manufactura aditiva]
- [Fuente PRBS / identificación de sistemas]

────────────────────────────────────────

APÉNDICES

A — Scripts MATLAB
- motor_bayes_opt_2.m
- fred_clasico_bo_hotend.m
- Script tfest() motor
- Script tfest() hotend

B — Firmware Arduino
- MAIN_F.ino

C — Datos Experimentales
- PRBS_Motor1/2/3.csv
- hotend60/100/180/220.csv
