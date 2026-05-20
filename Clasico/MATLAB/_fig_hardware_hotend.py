"""Genera Figura 4 (overlay hardware hotend) y Figura 5 (BO Fijo vs Adaptativo)."""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont

FIG_DIR = r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\Figuras'
CSV = {
    'normal': r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\PID originales.csv',
    'tuner':  r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\Pid tuner.csv',
    'bo':     r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Adaptativo\BO fijo.csv',
    'adapt':  r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\BO adaptativo.csv',
}


def load(path):
    df = pd.read_csv(path)
    t = df['Time_s'].values
    col = 'Temperatura_Hotend' if 'Temperatura_Hotend' in df.columns else 'Temp_Hotend'
    y = df[col].values
    return t - t[0], y


def stamp(path, text, color):
    img = Image.open(path).convert('RGB')
    w, h = img.size
    BH = 48
    out = Image.new('RGB', (w, h + BH), 'white')
    d = ImageDraw.Draw(out)
    d.rectangle([0, 0, w, BH], fill=color)
    font = None
    for fp in (r'C:\Windows\Fonts\arialbd.ttf', r'C:\Windows\Fonts\arial.ttf'):
        if os.path.exists(fp):
            font = ImageFont.truetype(fp, 28)
            break
    if font is None:
        font = ImageFont.load_default()
    bb = d.textbbox((0, 0), text, font=font)
    tw, th = bb[2] - bb[0], bb[3] - bb[1]
    d.text(((w - tw) // 2, (BH - th) // 2 - bb[1]), text, fill='white', font=font)
    out.paste(img, (0, BH))
    out.save(path)


# ---- Figura 4: overlay hardware hotend (3 esquemas) ----
fig, ax = plt.subplots(figsize=(10, 5))
for key, label, color in [('normal', 'PID Normal', 'tab:blue'),
                           ('tuner', 'PID Tuner', 'tab:orange'),
                           ('bo', 'BO Fijo', 'tab:red')]:
    t, y = load(CSV[key])
    ax.plot(t, y, label=label, linewidth=1.3)
ax.axhline(200, color='k', linestyle='--', linewidth=1, label='Setpoint 200 °C')
ax.set_xlabel('Tiempo (s)')
ax.set_ylabel('Temperatura Hotend (°C)')
ax.set_title('Respuesta Hotend en Al-FrED0 físico — 3 esquemas')
ax.legend(loc='lower right')
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 350)    # recortar cola plana: ~50 s despues del asentamiento
ax.set_ylim(50, 210)   # eje Y desde 50 C
p4 = os.path.join(FIG_DIR, 'fig4_hardware_hotend_overlay.png')
fig.savefig(p4, dpi=150, bbox_inches='tight')
plt.close(fig)
stamp(p4, 'DATOS DE HARDWARE (Al-FrED0)', (39, 110, 39))
print('OK Figura 4:', p4)

# ---- Figura 5: BO Adaptativo (con perturbación) ----
fig, ax = plt.subplots(figsize=(10, 5))
t, y = load(CSV['adapt'])
ax.plot(t, y, color='tab:red', linewidth=1.3, label='BO Adaptativo')
ax.axhline(200, color='k', linestyle='--', linewidth=1, label='Setpoint 200 °C')
ax.axhline(220, color='gray', linestyle='--', linewidth=1, label='Setpoint 220 °C')
ax.set_xlabel('Tiempo (s)')
ax.set_ylabel('Temperatura Hotend (°C)')
ax.set_title('Hotend con BO Adaptativo — control y respuesta a perturbación')
ax.legend(loc='lower right')
ax.grid(True, alpha=0.3)
p5 = os.path.join(FIG_DIR, 'fig5_hardware_hotend_adaptativo.png')
fig.savefig(p5, dpi=150, bbox_inches='tight')
plt.close(fig)
stamp(p5, 'DATOS DE HARDWARE (Al-FrED0)', (39, 110, 39))
print('OK Figura 5:', p5)
print('Hecho.')
