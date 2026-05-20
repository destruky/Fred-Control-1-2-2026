"""Genera Figura 4b — Zoom del error estacionario del hotend (3 esquemas)."""
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
}
SETPOINT = 200.0


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


fig, ax = plt.subplots(figsize=(10, 5))
schemes = [
    ('normal', 'PID Normal', 'tab:blue'),
    ('tuner',  'PID Tuner',  'tab:orange'),
    ('bo',     'BO Fijo',    'tab:red'),
]

for key, label, color in schemes:
    t, y = load(CSV[key])
    # Tomar ultimos 60 s para zoom
    mask = t >= (t[-1] - 60)
    t_ss = t[mask]
    y_ss = y[mask]
    media = np.mean(y_ss)
    ess = media - SETPOINT
    ax.plot(t_ss, y_ss, color=color, linewidth=1.4,
            label=f'{label}  (media={media:.2f} °C, ESS={ess:+.2f} °C)')

ax.axhline(SETPOINT, color='k', linestyle='--', linewidth=1, label='Setpoint 200 °C')
ax.set_xlabel('Tiempo (s) — últimos 60 s de régimen estable')
ax.set_ylabel('Temperatura Hotend (°C)')
ax.set_title('Figura 4b — Zoom del error estacionario en régimen permanente')
ax.set_ylim(197, 203)
ax.legend(loc='upper right', fontsize=9)
ax.grid(True, alpha=0.3)
out_path = os.path.join(FIG_DIR, 'fig4b_hardware_hotend_zoom_ess.png')
fig.savefig(out_path, dpi=150, bbox_inches='tight')
plt.close(fig)
stamp(out_path, 'DATOS DE HARDWARE (Al-FrED0)', (39, 110, 39))
print('OK Figura 4b:', out_path)
