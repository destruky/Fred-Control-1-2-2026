Control LQR Adaptativo basado en Red Neuronal para Al-FrED0

Objetivo:
Comparar tres esquemas de control, K-gain, LQR fijo y LQR adaptativo, para motor DC y hotend del Al-FrED0, usando redes neuronales para obtener el modelo lineal del sistema.

Específico:
Se busca superar las limitaciones de los controladores convencionales ante la dinámica cambiante del Al-FrED0. Se entrenan redes neuronales que capturan el comportamiento dinámico del motor DC y del hotend. Mediante linealización por Jacobiano se obtienen las matrices de espacio de estados para diseñar un regulador LQR óptimo. El resultado es una comparativa progresiva entre K-gain, LQR fijo y LQR adaptativo, que actualiza el modelo conforme cambia el punto de operación.

Medible:
Se reportan las ganancias de cada esquema de control para ambas plantas, validadas en Simulink. La comparativa documentará error en estado estacionario, tiempo de establecimiento y rechazo de perturbaciones, en simulación y en el extrusor físico.

Alcanzable:
El proyecto abarca entrenamiento de modelos neuronales, diseño LQR en MATLAB, validación en Simulink e implementación en Arduino Mega 2560. Se cuenta con datos experimentales reales e infraestructura Python/PyTorch y MATLAB disponible.

Relevante:
La metodología es aplicable a sistemas con comportamiento no lineal donde los modelos físicos son difíciles de obtener, con impacto en manufactura aditiva, robótica y control de procesos industriales.

Metodología:
Se entrenarán redes neuronales con datos experimentales. Por linealización Jacobiana se obtendrán las matrices A, B, C, D, con las cuales se resolverá la ecuación de Riccati discreta en MATLAB. Los tres esquemas K-gain, LQR fijo y LQR adaptativo se validarán en Simulink y se implementarán en firmware Arduino para comparación en operación real.

Equipo:
PM: Rodrigo Quintero Casso A01199249
Darío Gael Taboada Serna A00841826
Eugenio Alonso Rodríguez Monsivais A00842257
Diego Sanchez Tiznado A00844234
Eduardo Mateo Murillo Andrade A00842099
Adrián Oswaldo Salazar González A00838435