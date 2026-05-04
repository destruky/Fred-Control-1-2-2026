# FrED-TEC: Control LQR Fijo (Baseline Definitivo)

## 📌 Resumen Técnico
Esta carpeta contiene la implementación en C++ del controlador **LQR Fijo** (Realimentación de Estados Discreto) para el proyecto FrED (Filament Extruder Device). 

Este repositorio funciona como el **Baseline (Punto de Referencia)** oficial para el artículo de investigación (paper) a presentar en Expo Ingenierías. Su objetivo es aislar la matemática de control moderno para comparar su **esfuerzo computacional, tiempo de establecimiento, error en estado estacionario y rechazo a perturbaciones** frente al control clásico (PID) y la propuesta principal de LQR Adaptativo.

⚠️ **Nota de Lazo Cerrado:** Aunque las ganancias de este controlador se calculan *offline* en MATLAB y son estáticas ("Fijas"), el sistema opera en **Lazo Cerrado** discreto estricto ($T_s = 100\text{ms}$), corrigiendo la señal de control continuamente mediante la ley $u(k) = N_{bar}r(k) - Kx(k)$.

---

## 🧠 Arquitectura Matemática (Actualización)

Para mantener la compatibilidad con los modelos de espacio de estados generados por las Redes Neuronales (Identificación de Sistemas), este controlador utiliza una estructura de **Companion Form** con registros de desplazamiento (*Shift Registers*):

*   **Hotend (6 Estados):** El vector de estados $x(k)$ se compone de la temperatura actual, 2 temperaturas históricas y 3 señales de control PWM históricas.
*   **Motor DC (11 Estados):** El vector de estados $x(k)$ utiliza una ventana $W=5$, almacenando las RPM actuales, 5 RPM históricas y 5 señales PWM históricas.

---

## 📂 Inventario de Archivos

*   `LQR_FIJO_MAIN.ino`: Núcleo de ejecución. Mantiene el tiempo de muestreo a 10Hz, ejecuta los cálculos matriciales y gestiona la telemetría bidireccional con la interfaz en Python.
*   `LQR_HOTEND.h`: Lógica matricial, inicialización suave y restricciones de seguridad térmica (0-250 °C) para el bloque extrusor.
*   `LQR_MOTORDC.h`: Lógica matricial, lectura de interrupciones del encoder y cálculo restrictivo de PWM (zona muerta > 30) para el Motor DC.
*   `PIN_MAP.h`: Mapeo físico de hardware para la placa Arduino Mega + RAMPS 1.4.
*   `AlFrED0_GUI.py`: Interfaz Gráfica de Usuario oficial para monitoreo, envío de setpoints y exportación de datos CSV para el reporte.

---

## 🚀 Guía de Implementación (Para el Equipo)

Este código está estructurado para operar inmediatamente, pero requiere la inyección del modelo físico extraído. Sigue estos pasos antes de flashear el Arduino Mega:

### 1. Obtener las Matrices $K$ y $N_{bar}$
1. Ejecuta los scripts de entrenamiento de redes neuronales en Python ubicados en `Moderno/NN/` para generar los archivos `.mat`.
2. Abre MATLAB y ejecuta los scripts de diseño (ej. `fred_lqr_hotend_design.m` y `design_motor_lqr.m`).
3. Anota los vectores resultantes $K$ (1x6 para el hotend, 1x11 para el motor) y sus respectivos escalares $N_{bar}$.

### 2. Inyectar Ganancias en el Firmware
1. Abre `LQR_HOTEND.h` y localiza las variables `K_H[6]` y `Nbar_H`. Reemplaza los ceros con los valores de MATLAB.
2. Abre `LQR_MOTORDC.h` y localiza las variables `K_M[11]` y `Nbar_M`. Reemplaza los ceros con los valores de MATLAB.

### 3. Operación y Recolección de Datos
1. Compila y sube la carpeta `LQR_FIJO_MAIN` al Arduino Mega.
2. Ejecuta `AlFrED0_GUI.py`.
3. Para fines del *paper*: Utiliza el botón **Exportar CSV** tras aplicar pruebas de escalón (setpoint) y pruebas de perturbación mecánica/térmica. Estos datos serán la validación de la robustez del sistema.