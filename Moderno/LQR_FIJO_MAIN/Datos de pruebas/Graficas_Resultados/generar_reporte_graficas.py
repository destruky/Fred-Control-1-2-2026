import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 1. Configuración de rutas
# Como el script está dentro de 'Graficas_Resultados', la base de los CSV es un nivel arriba
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SCRIPT_DIR) # Esto sube a 'Datos de pruebas'
OUTPUT_DIR = SCRIPT_DIR # Las gráficas se guardan donde está el script

# Crear subcarpetas de salida si no existen
os.makedirs(os.path.join(OUTPUT_DIR, "Motor"), exist_ok=True)
os.makedirs(os.path.join(OUTPUT_DIR, "Hotend"), exist_ok=True)

# Estilo de la gráfica
plt.style.use('seaborn-v0_8-whitegrid')

def graficar_experimento_motor(archivo_csv, tipo_prueba, titulo, ylabel):
    ruta_completa = os.path.join(BASE_DIR, archivo_csv)
    if not os.path.exists(ruta_completa):
        print(f"Archivo no encontrado: {archivo_csv}")
        return
    
    df = pd.read_csv(ruta_completa)
    
    # La GUI exporta 1 fila cada 250ms (0.25 segundos)
    # Reconstruimos la escala de tiempo real
    df['Tiempo_s'] = df.index * 0.25 
    
    plt.figure(figsize=(10, 5), dpi=300)
    plt.plot(df['Tiempo_s'], df['RPM_Motor_DC'], color='#2980b9', linewidth=2, label='LQR Fijo Real')
    
    # Reconstrucción del Setpoint y sombreados según la metodología
    if tipo_prueba == 'Step':
        # 10s en espera (0), luego 20 RPM
        df['Setpoint'] = np.where(df['Tiempo_s'] < 10, 0, 20)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#e74c3c', linestyle='--', linewidth=1.5, label='Setpoint')
        
    elif tipo_prueba == 'Pert':
        # 10s espera, 60s en 20 RPM, a los 70s inicia la perturbación de 60s
        df['Setpoint'] = np.where(df['Tiempo_s'] < 10, 0, 20)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#e74c3c', linestyle='--', linewidth=1.5, label='Setpoint')
        plt.axvspan(70, 130, color='gray', alpha=0.2, label='Perturbación (Freno manual)')
        
    elif tipo_prueba == 'Change':
        # 10s espera, y cambios cada minuto (60s)
        condiciones = [
            (df['Tiempo_s'] < 10),
            (df['Tiempo_s'] >= 10) & (df['Tiempo_s'] < 70),    # 20 RPM
            (df['Tiempo_s'] >= 70) & (df['Tiempo_s'] < 130),   # 35 RPM
            (df['Tiempo_s'] >= 130) & (df['Tiempo_s'] < 190),  # 50 RPM
            (df['Tiempo_s'] >= 190) & (df['Tiempo_s'] < 250),  # 35 RPM
            (df['Tiempo_s'] >= 250)                            # 20 RPM
        ]
        valores = [0, 20, 35, 50, 35, 20]
        df['Setpoint'] = np.select(condiciones, valores, default=0)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#e74c3c', linestyle='--', linewidth=1.5, label='Setpoint Dinámico')

    plt.title(titulo, fontsize=14, fontweight='bold')
    plt.xlabel('Tiempo (s)', fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.legend(loc='lower right', fontsize=10)
    plt.tight_layout()
    
    # Guardar automáticamente en Graficas_Resultados/Motor
    nombre_salida = os.path.join(OUTPUT_DIR, f"Motor/Grafica_Motor_{tipo_prueba}.png")
    plt.savefig(nombre_salida)
    plt.close()
    print(f"✓ Gráfica guardada: {nombre_salida}")

def graficar_experimento_hotend(archivo_csv, tipo_prueba, titulo, ylabel):
    ruta_completa = os.path.join(BASE_DIR, archivo_csv)
    if not os.path.exists(ruta_completa):
        print(f"Archivo no encontrado: {archivo_csv}")
        return
    
    df = pd.read_csv(ruta_completa)
    df['Tiempo_s'] = df.index * 0.25
    
    # Obtener temperatura ambiente inicial
    temp_amb = df['Temperatura_Hotend'].iloc[0] if len(df) > 0 else 25
    
    plt.figure(figsize=(10, 5), dpi=300)
    plt.plot(df['Tiempo_s'], df['Temperatura_Hotend'], color='#c0392b', linewidth=2, label='LQR Fijo Real')
    
    # Reconstrucción de Setpoints y sombreados
    if tipo_prueba == 'Step':
        # 10s espera, luego 190 C
        df['Setpoint'] = np.where(df['Tiempo_s'] < 10, temp_amb, 190)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#2c3e50', linestyle='--', linewidth=1.5, label='Setpoint')
        
    elif tipo_prueba == 'Pert':
        # 10s espera, 9 min (540s) a 190C, a los 550s ventilador 100% por 4 min (240s)
        df['Setpoint'] = np.where(df['Tiempo_s'] < 10, temp_amb, 190)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#2c3e50', linestyle='--', linewidth=1.5, label='Setpoint')
        plt.axvspan(550, 790, color='blue', alpha=0.15, label='Perturbación (Ventilador 100%)')
        
    elif tipo_prueba == 'Change':
        # 10s espera, 7m (420s)@150C, 5m (300s)@190C, 6m (360s)@230C
        condiciones = [
            (df['Tiempo_s'] < 10),
            (df['Tiempo_s'] >= 10) & (df['Tiempo_s'] < 430),
            (df['Tiempo_s'] >= 430) & (df['Tiempo_s'] < 730),
            (df['Tiempo_s'] >= 730)
        ]
        valores = [temp_amb, 150, 190, 230]
        df['Setpoint'] = np.select(condiciones, valores, default=temp_amb)
        plt.plot(df['Tiempo_s'], df['Setpoint'], color='#2c3e50', linestyle='--', linewidth=1.5, label='Setpoint Dinámico')

    plt.title(titulo, fontsize=14, fontweight='bold')
    plt.xlabel('Tiempo (s)', fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.legend(loc='lower right', fontsize=10)
    plt.tight_layout()
    
    # Guardar automáticamente en Graficas_Resultados/Hotend
    nombre_salida = os.path.join(OUTPUT_DIR, f"Hotend/Grafica_Hotend_{tipo_prueba}.png")
    plt.savefig(nombre_salida)
    plt.close()
    print(f"✓ Gráfica guardada: {nombre_salida}")


if __name__ == "__main__":
    print("Generando gráficas a partir de archivos CSV...")
    
    # Procesamiento de Pruebas del Motor
    graficar_experimento_motor("LQRF_Motor_Step.csv", "Step", "Respuesta Escalón - Motor DC (LQR Fijo)", "Velocidad (RPM)")
    graficar_experimento_motor("LQRF_Motor_Pert.csv", "Pert", "Rechazo a Perturbaciones - Motor DC", "Velocidad (RPM)")
    graficar_experimento_motor("LQRF_Motor_Change.csv", "Change", "Seguimiento de Trayectoria - Motor DC", "Velocidad (RPM)")

    # Procesamiento de Pruebas del Hotend
    graficar_experimento_hotend("LQRF_Hotend_Step.csv", "Step", "Respuesta Escalón - Hotend (LQR Fijo)", "Temperatura (°C)")
    graficar_experimento_hotend("LQRF_Hotend_Pert.csv", "Pert", "Rechazo a Perturbaciones Térmicas - Hotend", "Temperatura (°C)")
    graficar_experimento_hotend("LQRF_Hotend_Change.csv", "Change", "Seguimiento Dinámico - Hotend", "Temperatura (°C)")
    
    print("¡Proceso completado exitosamente!")