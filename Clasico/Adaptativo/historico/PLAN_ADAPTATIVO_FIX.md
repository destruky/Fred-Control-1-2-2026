# Plan — Arreglar el módulo BO Adaptativo (FrED-TEC Clásico)

Objetivo: que el worker adaptativo identifique modelos válidos y resintonice PIDs
de forma confiable, para producir **Tabla 7** y **Figura 5** del reporte.

---

## Phase 0 — Diagnóstico (COMPLETADO)

### Evidencia (prueba1.csv, 2026-05-15)
- Duración 538 s, hotend 21 °C → 197.5 °C (llegó al setpoint).
- **0 retuneos** hotend, **0 retuneos** motor (columnas `Kp_*` todas NaN).
- Motor RPM ∈ [-2.81, 0.51] → encoder lee ruido, motor no gira.

### Causa raíz #1 — Modelo ARX inestable → FIT = −12000 %
- `sysid.py:67` — `identify_arx` resuelve mínimos cuadrados sin restricción
  (`np.linalg.lstsq`). Con datos mal condicionados (PWM saturado constante, o
  ruido en estado estable) el coeficiente `a1` puede dar `|polo| > 1`.
- `sysid.py:79` — el FIT% se calcula con `simulate_arx` (simulación libre / free-run).
- `sysid.py:116-128` — `simulate_arx` es recursivo: `y[k] = -a·y_hist + b·u_hist`.
  Si el polo es inestable, **la simulación diverge exponencialmente** → FIT → −∞.
- El hotend es **estable en lazo abierto** (se enfría solo). Cualquier modelo con
  polo inestable está físicamente mal y debe rechazarse.

### Causa raíz #2 — Falta de excitación persistente
- Durante calentamiento: PWM saturado en 255 (entrada constante) → ARX degenerada.
- En estado estable a 200 °C: PWM solo modula por ruido del termistor → SNR baja.
- ARX necesita **excitación** (la entrada debe variar de forma informativa).
- `BO_Adaptativo_Worker.py:192-202` ya tiene un gate de excitación
  (`u_std<10`, `u_mean∈[15,240]`, `y_std<1`) — correcto, pero implica que el
  worker **nunca** identifica salvo durante un transitorio real.

### Causa raíz #3 — Worker arranca sin modelo
- `AlFrED0_GUI_Adaptativo.py:433` — `BOAdaptativoWorker('hotend')` se crea **sin**
  `initial_model`. El worker (`BO_Adaptativo_Worker.py:96-98`) arranca con
  `_has_model=False` y depende 100 % de la identificación online, que falla.
- Existe un modelo offline válido y estable en
  `Clasico/MatLab/fred_bo_hotend_results.json` (`num`, `den`, FIT≈42 %).

### Causa raíz #4 — Motor sin señal
- RPM ≈ 0 → motor no gira o encoder desconectado. Hardware, independiente del worker.

### APIs reales confirmadas (no inventar)
| Símbolo | Ubicación | Firma / nota |
|---|---|---|
| `identify_arx(u,y,na,nb,d=1)` | `sysid.py:23` | retorna `(num, den, fit_pct)` |
| `identify_hotend(u,y,Ts=0.1)` | `sysid.py:177` | llama `identify_arx(na=1,nb=1,d=2)` |
| `identify_motor(u,y,Ts=0.1)` | `sysid.py:172` | llama `identify_arx(na=2,nb=2,d=1)` |
| `simulate_arx(num,den,u,d=1)` | `sysid.py:91` | free-run; **diverge si modelo inestable** |
| `DriftDetector.check(num,den,u,y)` | `drift_detector.py:38` | retorna `(has_drift, nrmse_pct)` |
| `BOAdaptativoWorker(planta, initial_model=None)` | `BO_Adaptativo_Worker.py:77` | `initial_model=(num,den)` |
| `optimize_pid(num,den,Ts,planta,x0,n_calls)` | `bo_pid.py:136` | retorna dict con `Kp,Ki,Kd,itae,success` |

### Anti-patrones a evitar
- NO usar la FIT de simulación libre para aceptar/rechazar un modelo (diverge).
- NO asumir que un setpoint-change (200→220) demuestra adaptación: cambia la
  *referencia*, no la *planta*. Para drift real hay que cambiar la **planta**
  (encender el ventilador, soplar aire frío, carga térmica).
- NO inventar parámetros en `pidtune`/`gp_minimize`/`lstsq` fuera de su firma.

---

## Phase 1 — Identificación robusta (`sysid.py`)

**Qué implementar** (editar `identify_arx`, `sysid.py:23-88`):

1. **Check de estabilidad** tras calcular `den` (después de `sysid.py:72`):
   ```python
   poles = np.roots(den)
   estable = len(poles) == 0 or np.max(np.abs(poles)) < 0.999
   ```
2. **Regularización ridge** en mínimos cuadrados (reemplazar `sysid.py:67`):
   en vez de `lstsq(Phi, Y)`, resolver
   `theta = np.linalg.solve(Phi.T@Phi + lam*np.eye(n), Phi.T@Y)`
   con `lam = 1e-3 * np.trace(Phi.T@Phi)/n`. Mantiene coeficientes acotados.
3. **FIT de predicción a 1 paso** para aceptar/rechazar (no la de simulación):
   `y_pred_1 = Phi @ theta` (ya es la predicción 1-paso, usa `y` real en regresores).
   `fit_1step = 100*(1 - norm(Y - y_pred_1)/norm(Y - mean(Y)))`.
   - Retornar `fit_pct = fit_1step` si `estable`, si no `fit_pct = -inf`.
   - Conservar la FIT de simulación libre solo como dato secundario opcional.

**Verificación:**
- `python -c "from sysid import identify_hotend; ..."` con datos del CSV →
  FIT debe ser un número finito (no −12000 %).
- Modelo retornado debe tener `max(|roots(den)|) < 1`.
- Grep: `grep -n "np.roots" sysid.py` debe encontrar el check nuevo.

**Anti-patrón:** no eliminar `simulate_arx` — `DriftDetector` lo usa; solo dejar
de usarlo para la decisión de aceptar/rechazar.

---

## Phase 2 — Worker arranca con modelo offline (`AlFrED0_GUI_Adaptativo.py`)

**Qué implementar:**

1. Al crear el worker hotend (`AlFrED0_GUI_Adaptativo.py:433`), cargar el modelo
   offline y pasarlo como `initial_model`:
   ```python
   import json
   _p = Path(__file__).parent.parent / "MatLab" / "fred_bo_hotend_results.json"
   _m = json.load(open(_p))
   modelo_hotend = (_m['num'], _m['den'])
   self.worker_hotend = BOAdaptativoWorker('hotend', initial_model=modelo_hotend)
   ```
2. Con `initial_model`, el worker (`BO_Adaptativo_Worker.py:93-95`) arranca con
   `_has_model=True` → puede correr BO desde el primer ciclo con un modelo válido.
3. La identificación online solo se dispara cuando `DriftDetector` detecta drift
   **y** la nueva identificación pasa estabilidad + FIT (Phase 1).

**Verificación:**
- Arrancar GUI, marcar Adaptativo Hotend, Iniciar control → en el primer ciclo
  con datos el log debe mostrar BO corriendo sobre el modelo offline (no
  `Modelo inicial rechazado`).
- Grep: `grep -n "initial_model" AlFrED0_GUI_Adaptativo.py` encuentra la carga.

**Anti-patrón:** no hardcodear `num`/`den` en el código; leerlos del JSON.

---

## Phase 3 — Diagnóstico motor (hardware, paralelo)

**Qué hacer:**
1. Verificar conexión física del encoder (C1=pin18, C2=pin19) al RAMPS.
2. Con la GUI, mandar `DCSPEED:30` + `ACTUATE:1001` y observar `RPM_Motor`.
3. Si sigue en ≈0: probar otro canal de encoder o confirmar alimentación del motor.

**Verificación:** `RPM_Motor` en el CSV debe subir a >5 RPM con PWM>30.

**Nota:** si el encoder no se recupera hoy, **diferir motor** y entregar solo
hotend (decisión ya tomada antes). No bloquea Tabla 7 hotend.

---

## Phase 4 — Prueba física del adaptativo (procedimiento)

**Pre-requisitos:** Phase 1 y 2 aplicadas; Arduino `MAIN_F_Adaptativo.ino`
re-flasheado (Kp_H=60); hotend reparado.

**Procedimiento:**
1. Abrir GUI adaptativa, marcar ✅ Adaptativo Hotend, setpoint 200 °C.
2. Iniciar control → esperar ~5 min a 200 °C estable.
3. **Perturbación que cambia la PLANTA** (no solo la referencia):
   - **A.** Encender el ventilador (`pinFan=8`) → aumenta pérdida de calor →
     cambia dinámica térmica → drift detectado → re-id → BO re-sintoniza.
   - **B.** Apagar ventilador 2 min después → segunda transición.
4. Observar log: `DRIFT detectado` → `Modelo actualizado (FIT=X%)` → `Nuevas
   ganancias Kp=…`.
5. Exportar CSV con el botón "Exportar CSV" → guardar como
   `Clasico/Resultados/hotend_adaptativo.csv`.

**Verificación:** el CSV debe tener ≥2 filas con `Kp_Hotend` no-NaN (retuneos).

**Anti-patrón:** no usar solo cambios de setpoint como "perturbación" — eso
prueba tracking, no adaptación. La perturbación debe alterar G(z).

---

## Phase 5 — Procesar CSV → Tabla 7 + Figura 5

**Qué hacer:**
1. Script Python: cargar `hotend_adaptativo.csv`, calcular por tramo
   (pre-perturbación / post-perturbación): ITAE, tiempo de recuperación,
   sobretiro, error estacionario.
2. Comparar contra el tramo equivalente del BO Fijo (`BO_fijo CSV.csv`).
3. Graficar temperatura vs tiempo marcando instantes de perturbación y de
   retuneo → guardar `Clasico/Resultados/Figuras/hw_hotend_adaptativo.png`.
4. Llenar **Tabla 7** y referenciar **Figura 5** en
   `Clasico/Resultados_Reporte_Clasico.md`.

**Verificación:** Tabla 7 sin `[PENDIENTE]`; figura existe en disco.

---

## Phase 6 — Verificación final

1. `python -c "import sysid, drift_detector, bo_pid, BO_Adaptativo_Worker"` —
   sin errores de import.
2. Grep anti-patrones:
   - `grep -n "np.roots" sysid.py` → check de estabilidad presente.
   - `grep -n "initial_model" AlFrED0_GUI_Adaptativo.py` → carga del modelo offline.
3. Re-correr prueba corta: el log debe mostrar al menos un ciclo BO exitoso
   sobre el modelo offline y, tras perturbación con ventilador, una re-id con
   FIT finito > 30 %.
4. Confirmar Tabla 7 y Figura 5 completas en el reporte.

---

## Resumen de archivos a tocar

| Archivo | Phase | Cambio |
|---|---|---|
| `Clasico/Adaptativo/sysid.py` | 1 | check estabilidad + ridge + FIT 1-paso |
| `Clasico/Adaptativo/AlFrED0_GUI_Adaptativo.py` | 2 | pasar `initial_model` del JSON |
| Hardware (encoder/Arduino) | 3 | diagnóstico motor |
| `Clasico/Resultados/hotend_adaptativo.csv` | 4 | CSV de la prueba |
| `Clasico/Resultados_Reporte_Clasico.md` | 5 | Tabla 7 + Figura 5 |

## Riesgo / alcance realista
- Phases 1-2: ~30-45 min de código + verificación.
- Phase 4: depende de hardware (fuga hotend reparada, Arduino flasheado).
- Si el motor no se recupera (Phase 3), entregar solo hotend — no bloquea Tabla 7.
- El adaptativo demuestra valor real solo con perturbación que cambie la planta
  (ventilador), no con cambio de setpoint.
