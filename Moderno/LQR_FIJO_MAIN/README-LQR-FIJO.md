# FrED-TEC: Control LQR Fijo (Baseline)

## Descripción del Proyecto
Esta carpeta contiene la implementación "limpia" e independiente del controlador **LQR Fijo** (Realimentación de Estados) para el proyecto FrED (Filament Extruder Device) del Tecnológico de Monterrey.

Este código fue diseñado específicamente como **Baseline (Punto de Referencia)** para el artículo de investigación. Su propósito es servir como punto de comparación en términos de **esfuerzo computacional** y **precisión** frente al control reactivo clásico (PID) y frente a la propuesta principal del proyecto: el **LQR Adaptativo Basado en Datos**.

⚠️ **Nota Teórica Importante:** Aunque este controlador se denomina "Fijo" (porque sus ganancias se calcularon de forma *offline* y no cambian), es un sistema de **Lazo Cerrado**, ya que lee continuamente los sensores para ajustar la señal de control $u = N_{bar}r - Kx$.

---

## Arquitectura de Archivos

Para mantener la pureza del experimento y evitar conflictos con lógicas anteriores, la estructura se divide en los siguientes archivos:

* **`LQR_FIJO_MAIN.ino`**: El cerebro principal. Maneja la comunicación serial con la interfaz de Python, establece el tiempo de muestreo riguroso ($T_s = 100\text{ms}$) y manda a llamar a los controladores matemáticos. Ignora de forma segura cualquier parámetro PID enviado por la interfaz.
* **`LQR_HOTEND.h`**: Contiene la lógica del *Shift Register* térmico y la ecuación matricial para el bloque de aluminio. 
* **`LQR_MOTORDC.h`**: Contiene la lectura de interrupciones del encoder, cálculo de RPM y la ecuación matricial para la velocidad del motor.
* **`PIN_MAP.h`**: Mapeo físico de los pines para la placa Arduino Mega con la shield RAMPS 1.4.

---

## Instrucciones de Uso y Configuración

### Paso 1: Actualizar las Matrices $K$ (K-Gain)
Antes de subir el código, debes introducir el modelo matemático extraído de tus datos experimentales.
1. Abre `LQR_HOTEND.h` y reemplaza los valores de `K_LQR_H` y `Nbar_H` con los números obtenidos en el script de MATLAB para el extrusor.
2. Abre `LQR_MOTORDC.h` y reemplaza los valores de `K_LQR_M` y `Nbar_M` con los números obtenidos en MATLAB para el motor.

### Paso 2: Subir a la Placa
1. Asegúrate de que todos los archivos listados arriba estén dentro de una carpeta llamada EXACTAMENTE `LQR_FIJO_MAIN`.
2. Abre `LQR_FIJO_MAIN.ino` en el Arduino IDE.
3. Verifica que tienes instalada la librería `AccelStepper.h`.
4. Compila y sube el código a tu placa Arduino Mega.

### Paso 3: Operación
1. Conecta la fuente de poder de la RAMPS 1.4.
2. Abre y ejecuta la interfaz de usuario en Python (`AlFrED0_GUI.py`).
3. Usa la GUI para mandar *setpoints* de temperatura y velocidad. El Arduino utilizará exclusivamente matemáticas de Control Moderno para estabilizar los actuadores.
