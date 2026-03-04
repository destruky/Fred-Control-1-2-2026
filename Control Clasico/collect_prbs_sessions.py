#!/usr/bin/env python3
import serial
import time
import csv
from datetime import datetime
from pathlib import Path

class PRBSCollector:
    def __init__(self, port='COM5', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print(f"✓ Conectado a {port}")
    
    def collect_session(self, filename, duration_sec=600):
        """
        Recolectar una sesión PRBS
        duration_sec: duración en segundos (default 10 min = 600s)
        """
        print(f"\n{'='*60}")
        print(f"Recolectando: {filename}")
        print(f"Duración: {duration_sec}s ({duration_sec/60:.1f} min)")
        print(f"{'='*60}")
        
        data = []
        start_time = time.time()
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t_ms', 'pwm_cmd', 'rpm_actual', 'pwm_output'])
            
            while time.time() - start_time < duration_sec:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line and ',' in line:
                        writer.writerow(line.split(','))
                        elapsed = time.time() - start_time
                        
                        # Imprimir cada 30 segundos
                        if int(elapsed) % 30 == 0 and int(elapsed) > 0:
                            print(f"  {elapsed:.0f}s/{duration_sec}s - {line}")
                except Exception as e:
                    print(f"  Error: {e}")
                    pass
        
        print(f"✓ Sesión completada")
        return True
    
    def close(self):
        self.ser.close()

def main():
    collector = PRBSCollector(port='COM3')  # Cambiar si es necesario
    
    sessions = [
        # (filename, descripción, duración)
        ('prbs_motor_slow.csv', 'Motor PRBS Lento (5-8s period)', 600),
        ('prbs_motor_fast.csv', 'Motor PRBS Rápido (1-2s period)', 600),
        ('prbs_motor_extreme.csv', 'Motor PRBS Extremo (0-255 PWM)', 600),
        ('prbs_motor_valid.csv', 'Motor PRBS Validación (TEST)', 300),
    ]
    
    print("\n" + "="*60)
    print("RECOLECTOR DE SESIONES PRBS AVANZADO")
    print("="*60)
    print("⚠️  ASEGÚRATE DE QUE:")
    print("  1. Arduino está cargado con código PRBS correcto")
    print("  2. FrED está alimentado")
    print("  3. Serial Monitor está CERRADO")
    print("  4. Puerto COM es correcto")
    print("="*60)
    
    input("\nPresiona ENTER para comenzar la recolección...")
    
    for filename, desc, duration in sessions:
        print(f"\n📊 {desc}")
        collector.collect_session(filename, duration_sec=duration)
        
        # Pausa entre sesiones
        if sessions.index((filename, desc, duration)) < len(sessions) - 1:
            print("\n⏸️  Pausa de 30 segundos antes de la siguiente sesión...")
            time.sleep(30)
    
    collector.close()
    print("\n✅ TODAS LAS SESIONES COMPLETADAS")

if __name__ == '__main__':
    main()
