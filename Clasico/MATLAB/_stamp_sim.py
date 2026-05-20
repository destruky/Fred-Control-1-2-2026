"""Agrega una banda 'SIMULACIÓN' a las 6 figuras de simular_realista."""
import os
from PIL import Image, ImageDraw, ImageFont

D = r'c:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\Figuras'
FILES = [
    'sim_motor_normal.png', 'sim_motor_tuner.png', 'sim_motor_bo.png',
    'sim_hotend_normal.png', 'sim_hotend_tuner.png', 'sim_hotend_bo.png',
]
BANNER_H = 48
TEXT = 'FIGURA DE SIMULACION (MATLAB)'

# Fuente
font = None
for fp in (r'C:\Windows\Fonts\arialbd.ttf', r'C:\Windows\Fonts\arial.ttf'):
    if os.path.exists(fp):
        font = ImageFont.truetype(fp, 28)
        break
if font is None:
    font = ImageFont.load_default()

for f in FILES:
    p = os.path.join(D, f)
    if not os.path.exists(p):
        print('NO existe:', f)
        continue
    img = Image.open(p).convert('RGB')
    w, h = img.size
    out = Image.new('RGB', (w, h + BANNER_H), 'white')
    draw = ImageDraw.Draw(out)
    draw.rectangle([0, 0, w, BANNER_H], fill=(31, 78, 121))  # azul oscuro
    bbox = draw.textbbox((0, 0), TEXT, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text(((w - tw) // 2, (BANNER_H - th) // 2 - bbox[1]), TEXT,
              fill='white', font=font)
    out.paste(img, (0, BANNER_H))
    out.save(p)
    print('OK:', f)

print('Hecho — 6 figuras marcadas como SIMULACION.')
