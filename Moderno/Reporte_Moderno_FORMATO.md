Instituto Tecnológico y de Estudios Superiores de Monterrey — Campus Monterrey

Control LQR Adaptativo basado en Red Neuronal para el extrusor Al-FrED0

Docente: Dr. Erick Guadalupe Ramírez Cedillo — erickramce@tec.mx

Equipo:
Rodrigo Quintero Casso — A01199249
Eduardo Mateo Murillo Andrade — A00842099
Diego Sánchez Tiznado — A00844234
Darío Gael Taboada Serna — A00841826
Eugenio Alonso Rodríguez Monsivais — A00842257
Adrián Oswaldo Salazar González — A00838435

[FECHA] — Monterrey, Nuevo León

────────────────────────────────────────

RESUMEN

[Problema → solución (NN + linealización Jacobiana + LQR) → tres esquemas comparados (K-gain, LQR fijo, LQR adaptativo) → resultados numéricos (error estacionario, tiempo de establecimiento, rechazo de perturbaciones) → validación Simulink + hardware]

────────────────────────────────────────

1. INTRODUCCIÓN

[Por qué el Al-FrED0 necesita control moderno: la dinámica cambia con el punto de operación (temperatura, desgaste), haciendo que un PID estático sea insuficiente. Por qué redes neuronales: capturan comportamiento no lineal sin requerir modelo físico. Presentar la progresión K-gain → LQR fijo → LQR adaptativo como contribución principal.]

2. CONTEXTO GENERAL

[Al-FrED0: extrusor de filamento, dos plantas a controlar. Diferencia de dinámicas: τ_motor ≈ 0.2 s vs τ_hotend ≈ 100 s. Por qué los modelos físicos son difíciles de obtener para este sistema. Motivación para usar datos experimentales + redes neuronales.]

3. DELIMITACIÓN DEL OBJETO DE ESTUDIO

Este proyecto diseña e implementa tres esquemas de control para dos subsistemas del Al-FrED0:
- Motor DC de embobinado (PWM → RPM)
- Hotend (PWM → °C)

Los tres esquemas, en orden de complejidad creciente: K-gain individual, LQR fijo y LQR adaptativo basado en datos. Hardware fijo: Arduino Mega 2560 + RAMPS 1.4. Sin modificaciones mecánicas. Prohibido usar tfest(), Transfer Function block o PID Tuner.

4. PLANTEAMIENTO DEL PROBLEMA

[Los controladores convencionales (PID estático) no compensan la dinámica cambiante del Al-FrED0 ante variaciones de temperatura, material y desgaste. Un modelo físico preciso es difícil de obtener. Se necesita un enfoque basado en datos que capture el comportamiento no lineal y adapte el control en tiempo real.]

5. JUSTIFICACIÓN

[Por qué redes neuronales sobre modelos físicos: no requieren ecuaciones de la planta, aprenden de datos reales. Por qué LQR sobre PID: óptimo en el sentido de minimizar la función de costo J = Σ(xᵀQx + uᵀRu). Por qué adaptativo sobre fijo: el punto de linealización cambia con la operación. Relevancia en manufactura aditiva y sistemas no lineales.]

────────────────────────────────────────

6. MARCO TEÓRICO

6.1 Redes Neuronales para Identificación de Sistemas
[Arquitectura NARX (Nonlinear AutoRegressive with eXogenous inputs): la red predice la salida futura a partir de una ventana de entradas y salidas pasadas. Por qué es adecuada para sistemas dinámicos. Entrenamiento supervisado: minimización del error de predicción un paso (warmup) seguido de error multi-paso (NOE). Métricas: RMSE y R².]

6.2 Linealización por Jacobiano
[Concepto: dado el modelo no lineal representado por la NN, se obtiene una aproximación lineal local calculando el Jacobiano de la función de la red respecto a sus entradas. Resultado: matrices A, B, C, D del espacio de estados discreto en el punto de operación actual.]

Figura A. Diagrama de bloques del pipeline completo.
[DIAGRAMA: datos experimentales → entrenamiento NN → Jacobiano → A,B,C,D → dlqr() → K, Nbar → lazo cerrado en Arduino → planta → retroalimentación → (rama adaptativa: re-linealización periódica)]

6.3 Regulador LQR Discreto
[Función de costo J = Σ(xᵀQx + uᵀRu). Solución: ecuación de Riccati discreta → ganancia K óptima. Señal de control: u = Nbar·r − K·x. Rol de Q (penaliza estados) y R (penaliza esfuerzo de control). dlqr() en MATLAB.]

6.4 Tres Esquemas de Control

K-gain individual: ganancia estática calculada offline una sola vez. Baseline más simple, lazo cerrado, ganancias fijas permanentes.

LQR fijo: ganancias K calculadas offline con dlqr() usando el modelo linealizado en un punto de operación. Se flashean en el Arduino y no cambian durante la operación.

LQR adaptativo: el modelo se re-linealiza periódicamente con datos recientes. La ecuación de Riccati se resuelve en tiempo real (Python, hilo separado) y las nuevas ganancias K se envían al Arduino vía serial. Actualización cada 2 s.

6.5 Reducción de Orden — minreal()
[El hotend genera un sistema de 201 estados (companion form de la NN con W=100). minreal() en MATLAB elimina los estados no controlables/no observables, reduciendo a ~6 estados manejables para dlqr(). Necesario antes del diseño LQR del hotend.]

────────────────────────────────────────

7. OBJETIVO GENERAL

Diseñar e implementar un sistema de control LQR adaptativo basado en redes neuronales para el motor DC y el hotend del Al-FrED0, comparando tres esquemas de complejidad creciente (K-gain, LQR fijo, LQR adaptativo) en términos de error estacionario, tiempo de establecimiento y rechazo de perturbaciones.

8. OBJETIVOS ESPECÍFICOS

- Entrenar redes neuronales NARX en PyTorch para el motor DC y el hotend usando datos experimentales, alcanzando R² ≥ [XX] en ambas plantas.
- Obtener las matrices de espacio de estados A, B, C, D mediante linealización Jacobiana de las redes neuronales en el punto de operación.
- Diseñar los reguladores LQR (K-gain, fijo y adaptativo) usando dlqr() en MATLAB y validarlos en Simulink con el bloque State-Space.
- Implementar los tres esquemas de control en el firmware del Arduino Mega y validar en el Al-FrED0 físico.
- Comparar el desempeño de los tres esquemas en error estacionario, tiempo de establecimiento y rechazo de perturbaciones.

9. HIPÓTESIS

Si se entrenan redes neuronales que capturen la dinámica del motor DC y el hotend del Al-FrED0, y se linealiza por Jacobiano para diseñar un regulador LQR adaptativo, entonces el LQR adaptativo superará al LQR fijo y al K-gain en rechazo de perturbaciones y estabilidad ante cambios de punto de operación, manteniendo error estacionario menor a [XX]% en ambas plantas.

────────────────────────────────────────

10. METODOLOGÍA

10.1 Recolección de Datos

Motor DC — señal PRBS, niveles PWM {30, 60, 90, 120, 150}, dwell = 1.5 s:
- PRBS_Motor1.csv (entrenamiento)
- PRBS_Motor2.csv (validación)
- PRBS_Motor3.csv (prueba)

Hotend — escalones PWM {60, 100, 180, 220}, duración ≥ 5τ por nivel:
- hotend60/100/180/220.csv

Figura B. Señal PRBS aplicada al motor DC y respuesta de RPM medida.
[GRÁFICA: tiempo vs PWM (arriba) y tiempo vs RPM (abajo)]

Figura C. Escalones de PWM aplicados al hotend y respuesta de temperatura.
[GRÁFICA: tiempo vs PWM (arriba) y tiempo vs °C (abajo)]

10.2 Entrenamiento de Redes Neuronales

Motor DC — archivo: motor_sysid_nn.py
Arquitectura: Feedforward 12→32→32→1
Ventana temporal: W = 5 pasos
Entrada: [PWM(k-5..k), RPM(k-5..k)] → RPM(k+1)
Normalización: PWM/255, RPM/55
Punto de operación para linealización: PWM = 75, RPM = 28

| Métrica | Valor |
|---------|-------|
| RMSE | 0.54 RPM |
| R² | 0.9988 |

Hotend — archivo: hotend_sysid_nn.py
Arquitectura: MLP con Tanh, W = 100 pasos (10 s de historia)
Entrada: [PWM(k-100..k), temp(k-100..k)] → temp(k+1)
Entrenamiento en 2 fases: warmup 1-step + NOE multi-step (horizonte 20→300 pasos)

| Métrica | Valor |
|---------|-------|
| RMSE | [XX] °C |
| R² | [XX] |

Figura D. Predicción de la NN vs datos reales — Motor DC (set de prueba).
[GRÁFICA: RPM medida vs RPM predicha por la red]

Figura E. Predicción de la NN vs datos reales — Hotend (set de prueba).
[GRÁFICA: °C medida vs °C predicha por la red]

10.3 Linealización y Obtención de Matrices de Estado

Se calculó el Jacobiano de cada red neuronal en el punto de operación para obtener las matrices discretas A, B, C, D (Ts = 0.1 s).

Motor DC — salida: state_space_motor.mat
- Dimensiones: A(11×11), B(11×1), C(1×11), D(1×1)
- Companion form completa: 2W+1 = 11 estados
- Sistema estable en lazo abierto (eigenvalores dentro del círculo unitario)

Hotend — salida: state_space_hotend.mat
- Dimensiones originales: 201 estados (companion form, W=100)
- Reducción con minreal() en MATLAB → [XX] estados controlables
- Script: fred_lqr_hotend_design.m

10.4 Diseño LQR con dlqr()

Script motor: design_motor_lqr.m
Script hotend: fred_lqr_hotend_design.m

Matrices de ponderación seleccionadas:

| Parámetro | Motor | Hotend |
|-----------|-------|--------|
| Q | [MATRIZ/VALORES] | [MATRIZ/VALORES] |
| R | [VALOR] | [VALOR] |

Resultados:

| Esquema | Planta | K | Nbar |
|---------|--------|---|------|
| K-gain | Motor DC | [VALORES] | [XX] |
| K-gain | Hotend | [VALORES] | [XX] |
| LQR fijo | Motor DC | [VALORES] | [XX] |
| LQR fijo | Hotend | [VALORES] | [XX] |

10.5 Validación en Simulink

Modelos: Motorcontrol.slx | Hotendcontrol.slx
Bloque: State-Space (no Transfer Function, no PID)
Comparativa: K-gain vs LQR fijo vs LQR adaptativo

Configuración:
- Motor: escalón [XX] RPM, simulación [XX] s
- Hotend: escalón [XX] °C, simulación 800–1000 s

10.6 Implementación en Arduino — LQR Fijo

Firmware: LQRMAIN_F/LQR_FIJO_MAIN.ino — Arduino Mega 2560 + RAMPS 1.4
- Motor: pin 9, encoder pines 18/19, Ts = 100 ms
- Hotend: pin 10, termistor A13, shutdown > 255 °C
- Archivos: LQR_MOTORDC.h | LQR_HOTEND.h | PIN_MAP.h

Figura F. Estructura de archivos del firmware LQR fijo.
[DIAGRAMA: LQR_FIJO_MAIN.ino → LQR_MOTORDC.h / LQR_HOTEND.h / PIN_MAP.h]

10.7 Implementación del LQR Adaptativo

Arquitectura: hilo Python (QThread) + Arduino vía serial
- Hilo matemático: Adaptativo_Worker.py — ciclo de 2 s
- Ciclo: ventana de datos recientes → Jacobiano NN → A,B,C,D → dlqr() → nuevos K, Nbar → envío serial al Arduino
- GUI: AlFrED0_GUI_V2.py — monitoreo en tiempo real y envío de setpoints
- Dependencias: PyTorch, python-control, PyQt5, scipy

[Describir: cómo el Arduino recibe y aplica los nuevos K sin interrumpir el control.]

────────────────────────────────────────

11. TÉCNICAS Y HERRAMIENTAS

| Herramienta | Uso |
|-------------|-----|
| Python + PyTorch | Entrenamiento de redes neuronales NARX |
| MATLAB (Jacobiano) | Linealización de la NN → A, B, C, D |
| MATLAB dlqr() | Diseño de reguladores LQR |
| MATLAB minreal() | Reducción de orden del modelo hotend |
| Simulink (State-Space) | Validación de esquemas de control |
| Arduino IDE (C++) | Firmware LQR fijo |
| Python-control + PyQt5 | Hilo adaptativo + GUI serial |

12. INFRAESTRUCTURA Y RECURSOS

Hardware:
| Componente | Descripción |
|------------|-------------|
| Arduino Mega 2560 | Microcontrolador |
| RAMPS 1.4 | Shield de potencia |
| Motor DC + encoder | 4704 pulsos/rev, ~52 RPM máx |
| Hotend Artillery | Calentador + termistor Steinhart-Hart |
| Fuente 12V / 20A | Alimentación |

Software: Python 3.X + PyTorch | MATLAB R[XXXX] | Arduino IDE

────────────────────────────────────────

13. RESULTADOS

Tabla 1. Desempeño de las redes neuronales entrenadas.

| Planta | RMSE | R² | Estados (antes/después minreal) |
|--------|------|----|---------------------------------|
| Motor DC | 0.54 RPM | 0.9988 | 11 / — |
| Hotend | [XX] °C | [XX] | 201 / [XX] |


Tabla 2. Ganancias K y Nbar por esquema.

| Esquema | Planta | K | Nbar |
|---------|--------|---|------|
| K-gain | Motor DC | [VALORES] | [XX] |
| K-gain | Hotend | [VALORES] | [XX] |
| LQR fijo | Motor DC | [VALORES] | [XX] |
| LQR fijo | Hotend | [VALORES] | [XX] |


Tabla 3. Desempeño en Simulink — Motor DC.

| Métrica | K-gain | LQR Fijo | LQR Adaptativo |
|---------|--------|----------|----------------|
| T. establecimiento (s) | [XX] | [XX] | [XX] |
| Sobretiro (%) | [XX] | [XX] | [XX] |
| Error estacionario | [XX] | [XX] | [XX] |

Figura G. Respuesta escalón Simulink — Motor DC: K-gain vs LQR fijo vs LQR adaptativo.
[GRÁFICA]

Tabla 4. Desempeño en Simulink — Hotend.

| Métrica | K-gain | LQR Fijo | LQR Adaptativo |
|---------|--------|----------|----------------|
| T. establecimiento (s) | [XX] | [XX] | [XX] |
| Sobretiro (%) | [XX] | [XX] | [XX] |
| Error estacionario | [XX] | [XX] | [XX] |

Figura H. Respuesta escalón Simulink — Hotend: K-gain vs LQR fijo vs LQR adaptativo.
[GRÁFICA]


Tabla 5. Desempeño en hardware — Motor DC.

| Métrica | K-gain | LQR Fijo | LQR Adaptativo |
|---------|--------|----------|----------------|
| T. establecimiento (s) | [XX] | [XX] | [XX] |
| Sobretiro (%) | [XX] | [XX] | [XX] |
| Error estacionario | [XX] | [XX] | [XX] |
| Rechazo de perturbación | [XX] | [XX] | [XX] |

Tabla 6. Desempeño en hardware — Hotend.

| Métrica | K-gain | LQR Fijo | LQR Adaptativo |
|---------|--------|----------|----------------|
| T. establecimiento (s) | [XX] | [XX] | [XX] |
| Sobretiro (%) | [XX] | [XX] | [XX] |
| Error estacionario | [XX] | [XX] | [XX] |
| Rechazo de perturbación | [XX] | [XX] | [XX] |

Figura I. Respuesta motor DC en Al-FrED0 físico — comparativa tres esquemas.
[GRÁFICA]

Figura J. Respuesta hotend en Al-FrED0 físico — comparativa tres esquemas.
[GRÁFICA]

Figura K. LQR adaptativo ante cambio de punto de operación — actualización de K en tiempo real.
[GRÁFICA: K vs tiempo, mostrando las actualizaciones del hilo adaptativo]

────────────────────────────────────────

14. CONCLUSIONES

[1 — ¿El LQR adaptativo superó al fijo y al K-gain? Cifras concretas de las tablas.]
[2 — Diferencia entre plantas: motor converge rápido (11 estados, τ pequeña), hotend requiere minreal y simulación larga. Qué implica para el diseño.]
[3 — Limitaciones: hotend de 201 estados requiere minreal (pérdida de información), ciclo adaptativo de 2 s puede ser lento ante perturbaciones rápidas, latencia serial entre Python y Arduino.]
[4 — Trabajo futuro: reducir ciclo adaptativo, probar Q/R diferentes, extender a más puntos de operación, implementar en hardware más potente que Arduino.]

────────────────────────────────────────

15. REFERENCIAS

[APA — incluir:]
- Åström & Wittenmark (1995). Adaptive Control.
- Ljung (1999). System Identification: Theory for the User.
- Anderson & Moore (2007). Optimal Control: Linear Quadratic Methods.
- [Fuente redes neuronales para identificación de sistemas / NARX]
- [Fuente manufactura aditiva — Turner et al. (2014) o Gibson et al. (2021)]
- MATLAB Documentation — dlqr(), minreal()
- PyTorch Documentation

────────────────────────────────────────

APÉNDICES

A — Scripts Python (entrenamiento NN)
- motor_sysid_nn.py — red neuronal motor DC
- hotend_sysid_nn.py — red neuronal hotend

B — Scripts MATLAB (diseño LQR)
- design_motor_lqr.m — K y Nbar motor
- fred_lqr_hotend_design.m — K y Nbar hotend

C — Firmware Arduino
- LQR_FIJO_MAIN.ino
- LQR_MOTORDC.h | LQR_HOTEND.h | PIN_MAP.h

D — Software Adaptativo
- Adaptativo_Worker.py — hilo matemático LQR en tiempo real
- AlFrED0_GUI_V2.py — interfaz gráfica

E — Datos Experimentales
- PRBS_Motor1/2/3.csv — datos PRBS motor
- hotend60/100/180/220.csv — datos escalón hotend
