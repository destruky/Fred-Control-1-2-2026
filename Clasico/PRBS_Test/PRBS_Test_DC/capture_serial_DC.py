import serial
import serial.tools.list_ports
import time
import platform

def encontrar_puerto_arduino():
    puertos = serial.tools.list_ports.comports()
    for p in puertos:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if ("arduino" in desc) or ("usb serial" in desc) or ("ch340" in desc) or ("1a86" in hwid) or ("2341" in hwid):
            return p.device
    if platform.system() == "Windows": return "COM3"
    elif platform.system() == "Linux": return "/dev/ttyACM0"
    else: return "/dev/tty.usbmodem1101"

puerto = encontrar_puerto_arduino()
print(f"Conectando a {puerto}...")
ser = serial.Serial(puerto, 115200)
time.sleep(2)

nombre = input("Nombre del archivo (ej: prbs_motor_valid_v2): ") + ".csv"

print(f"Grabando en {nombre} — Ctrl+C para parar\n")

with open(nombre, 'w') as f:
    try:
        while True:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(line)
                f.write(line + '\n')
    except KeyboardInterrupt:
        print(f"\nListo. Datos guardados en {nombre}")

ser.close()