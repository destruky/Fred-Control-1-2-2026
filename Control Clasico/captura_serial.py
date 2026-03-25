# captura_serial.py
import serial, time

PORT   = 'COM4'      # ajusta si tu Arduino está en otro puerto
BAUD   = 115200
SECS   = 300         # 5 minutos

with serial.Serial(PORT, BAUD, timeout=1) as ser, \
     open('datos_motor.csv', 'w') as f:
    t_end = time.time() + SECS
    while time.time() < t_end:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
            f.write(line + '\n')

print("Captura terminada → datos_motor.csv")