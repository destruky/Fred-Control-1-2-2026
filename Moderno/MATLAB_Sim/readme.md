Simulación de Espacio de Estados - Motor y Hotend
Esta carpeta contiene los archivos necesarios para la simulación en MATLAB y Simulink de sistemas modelados en espacio de estados.

Archivos requeridos
Para ejecutar la simulación, es necesario contar con los siguientes tres archivos en el mismo directorio:

Archivo .slx: Modelo de bloques en Simulink.

Archivo .m: Script de inicialización y carga de parámetros.

Archivo .mat: Datos de variables y matrices.

Instrucciones de ejecución
Carga de archivos:
Abra el archivo .slx y el archivo .m. No ejecute ninguno de los archivos hasta completar los siguientes pasos.

Validación de ruta:
En el editor de MATLAB, revise el script .m y asegúrese de que la línea encargada de cargar el archivo .mat apunte al nombre y ubicación correctos. De ser necesario, corrija la ruta manualmente.

Inicialización del Workspace:
Ejecute el script .m. Esto cargará las matrices de estado y constantes necesarias en el espacio de trabajo de MATLAB.

Sincronización con Simulink:
Vuelva al modelo de Simulink y verifique que los nombres de las variables en los bloques coincidan con las variables generadas en el Workspace.

Simulación:
Inicie la simulación en Simulink. Los resultados podrán visualizarse directamente a través del bloque Scope.
