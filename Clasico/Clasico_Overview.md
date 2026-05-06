Control PID con Optimización Bayesiana para Al-FrED0

Objetivo:
Sintonizar automáticamente un controlador PID para motor DC y hotend del Al-FrED0 mediante Optimización Bayesiana, y ajustar sus ganancias en línea con ML.

Específico:
Se elimina la sintonización manual de ganancias PID mediante un flujo sistemático: identificación de G(s) con datos experimentales (PRBS para motor, escalón para hotend), búsqueda automatizada de Kp, Ki, Kd con Optimización Bayesiana minimizando ITAE, e implementación en Arduino. Como fase final, un módulo de aprendizaje automático ajusta las ganancias en tiempo real conforme cambia el punto de operación.

Medible:
Se reportan ganancias PID óptimas para motor DC (PWM→RPM) y hotend (PWM→°C), con comparación ITAE entre el baseline del PID Tuner de MATLAB y los valores propuestos por la optimización. Se documentan las G(s) identificadas, respuestas al escalón en Simulink, y el desempeño del PID adaptativo frente al estático.

Alcanzable:
El proyecto parte de datos experimentales recolectados del extrusor físico e infraestructura MATLAB/Simulink disponible. La implementación se realiza sobre firmware Arduino Mega 2560 con RAMPS 1.4 existente, sin cambios de hardware.

Relevante:
La metodología proporciona un procedimiento replicable de sintonización automática y adaptativa para plantas con dinámica no lineal, con aplicación en manufactura aditiva y control de procesos industriales.

Metodología:
Se identifican G(s) mediante tfest() en MATLAB. Se ejecuta Optimización Bayesiana minimizando ITAE en simulación discreta. Las ganancias se validan en Simulink con el bloque PID y se implementan en firmware Arduino. Finalmente, un módulo adaptativo resintoniza Kp, Ki, Kd en tiempo real ante cambios de operación.

Equipo
PM: Rodrigo Quintero Casso A01199249
Sergio René Castillo Cantú  A01723797
Mariana Ameyali Aguilar González A00844156
Hans Enrique Velarde Barrón A01286990
Adrián Oswaldo Salazar González A00838435