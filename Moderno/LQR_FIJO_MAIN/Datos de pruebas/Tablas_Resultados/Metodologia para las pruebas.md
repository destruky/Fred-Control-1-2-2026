# 📊 Protocolo de Pruebas Experimentales: Control LQR Fijo

Este documento establece el **estándar de experimentación estricto** para la recolección de datos utilizando el firmware de control moderno por realimentación de estados (**LQR Fijo / Servo-LQI**)[cite: 24, 25]. Para asegurar que los resultados sean estadísticamente comparables con los esquemas PID y LQR Adaptativo en el reporte final, todos los miembros del equipo deben seguir este protocolo al pie de la letra[cite: 26, 27].

---

## 📂 Organización de Archivos y Nomenclatura

Cada experimento debe exportarse inmediatamente al terminar utilizando el botón `📁 Exportar CSV` de la interfaz `AlFrED0_GUI_10.py`[cite: 19, 20]. Los archivos deben nombrarse bajo la siguiente convención rigurosa[cite: 24]:

`[CONTROL]_[PLANTA]_[PRUEBA].csv`

*   **[CONTROL]:** `LQRF` (Para nuestro equipo de LQR Fijo).
*   **[PLANTA]:** `Motor` o `Hotend`.
*   **[PRUEBA]:** `Step`, `Pert` o `Change`.

### Definición de Pruebas:
1.  **Step (Escalón):** Consiste en encender el actuador desde cero y evaluar su respuesta transitoria (tiempo de establecimiento y sobretiro) hasta alcanzar el estado estacionario[cite: 27].
2.  **Pert (Perturbación):** Consiste en someter al sistema a una carga externa destructiva una vez estabilizado, evaluando la robustez y capacidad de rechazo a perturbaciones del lazo cerrado[cite: 27].
3.  **Change (Cambio de Setpoint):** Consiste en variar las referencias dinámicamente en una misma sesión para evaluar la fidelidad de seguimiento (*tracking*) en diferentes regiones de operación[cite: 27].

---

## ⚡ Metodología de Ejecución Paso a Paso

> ⚠️ **Nota Crítica de Hardware:** No ejecutes las pruebas del motor y del hotend simultáneamente. Las caídas de voltaje de la fuente de poder de 12V afectarían el desempeño de los lazos de control viciando las lecturas.

### 🚗 1. Subsistema Motor DC
Asegurar que el Arduino Mega esté flasheado con `LQR_FIJO_MAIN_7.ino` y los buffers históricos estén limpios[cite: 20, 22]. El lazo discreto opera estrictamente a $T_s = 100\text{ms}$ (10 Hz)[cite: 20].

#### 🔹 Motor - Step (`LQRF_Motor_Step.csv`)
1.  Dejar el sistema **en espera (reposo) durante 10 segundos** para establecer el baseline del sensor.
2.  Encender el actuador `Motor DC` desde la interfaz.
3.  Mantener funcionando por **2 minutos** con un setpoint fijo de **20 RPM**.
4.  Apagar el actuador desde la GUI.
5.  Mantener en espera final por **10 segundos** y presionar `Exportar CSV`.

#### 🔹 Motor - Pert (`LQRF_Motor_Pert.csv`)
1.  Dejar el sistema **en espera durante 10 segundos**.
2.  Encender el actuador `Motor DC`.
3.  Mantener funcionando por **1 minuto** con un setpoint de **20 RPM**.
4.  Aplicar la perturbación mecánica por **1 minuto** (frenar físicamente el sistema de bobinado aplicando presión moderada con un dedo).
5.  Apagar el actuador.
6.  Mantener en espera final por **10 segundos** y presionar `Exportar CSV`.

#### 🔹 Motor - Change (`LQRF_Motor_Change.csv`)
1.  Dejar el sistema **en espera durante 10 segundos**.
2.  Encender el actuador `Motor DC`.
3.  Funcionar por **1 minuto** a **20 RPM** setpoint.
4.  Cambiar el slider a **35 RPM** setpoint y mantener por **1 minuto**.
5.  Cambiar el slider a **50 RPM** setpoint y mantener por **1 minuto**.
6.  Regresar el slider a **35 RPM** setpoint y mantener por **1 minuto**.
7.  Regresar el slider a **20 RPM** setpoint y mantener por **1 minuto**.
8.  Apagar el actuador.
9.  Mantener en espera final por **10 segundos** y presionar `Exportar CSV`.

---

### 🔥 2. Subsistema Hotend (Bloque Térmico)
Debido a la constante de tiempo térmica ($\tau \approx 100\text{s}$), estas pruebas requieren tiempos prolongados para registrar adecuadamente el estado estacionario. Verifique que la seguridad por *shutdown* térmico esté activa en `LQR_HOTEND_7.h`[cite: 21, 24].

#### 🔸 Hotend - Step (`LQRF_Hotend_Step.csv`)
1.  Dejar el sistema **en espera durante 10 segundos** (registro de temperatura ambiente).
2.  Encender el actuador `Heater` desde la interfaz.
3.  Mantener funcionando por **12 minutos** con un setpoint fijo de **190 °C**.
4.  Apagar el actuador desde la GUI.
5.  Dejar el sistema **en espera por 1 minuto** para observar la curva de enfriamiento inicial y presionar `Exportar CSV`.

#### 🔸 Hotend - Pert (`LQRF_Hotend_Pert.csv`)
1.  Dejar el sistema **en espera durante 10 segundos**.
2.  Encender el actuador `Heater`.
3.  Mantener funcionando por **9 minutos** con un setpoint de **190 °C** hasta que la acción integral se estabilice[cite: 21].
4.  Inyectar la perturbación térmica durante **4 minutos** encendiendo el ventilador (`Fan`) al **100%** de velocidad.
5.  Apagar absolutamente todos los actuadores.
6.  Dejar el sistema **en espera por 1 minuto** y presionar `Exportar CSV`.

#### 🔸 Hotend - Change (`LQRF_Hotend_Change.csv`)
1.  Dejar el sistema **en espera durante 10 segundos**.
2.  Encender el actuador `Heater`.
3.  Funcionar por **7 minutos** a un setpoint de **150 °C**.
4.  Subir el setpoint a **190 °C** y mantener funcionando por **5 minutos**.
5.  Subir el setpoint a **230 °C** (límite superior de operación segura) y mantener por **6 minutos**[cite: 21].
6.  Apagar el actuador.
7.  Dejar el sistema **en espera por 1 minuto** y presionar `Exportar CSV`.

---

## 📈 Checklist para Validación de Datos

Antes de dar un experimento por válido, abre el archivo `.csv` generado y comprueba lo siguiente:
*   [ ] ¿El archivo cuenta con todas las columnas correspondientes? (`Timestamp`, `Temperatura_Hotend`, `RPM_Motor_DC`, etc.)[cite: 19].
*   [ ] ¿Se visualiza el tiempo muerto de espera inicial de 10 segundos de manera limpia?
*   [ ] ¿Los tiempos de conmutación de setpoints coinciden con el protocolo?

Estos archivos CSV serán cargados en el script de post-procesamiento para graficar las curvas comparativas definitivas de la **Sección 13 (Resultados)** de nuestra entrega[cite: 27].