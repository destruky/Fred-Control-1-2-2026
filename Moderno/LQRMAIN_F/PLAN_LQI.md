# Plan: LQR FIJO Correcto (Servo-LQR / LQI) — Moderno/LQRMAIN_F

**Objetivo**: Convertir el LQR FIJO actual (`u = Nbar·r − K·x`, sin integrador) en un servo-LQR / LQI con acción integral, eliminando error en estado estacionario por mismatch modelo-planta. Arreglar zona muerta motor y consistencia de reset.

**Alcance**: Solo `Moderno/LQRMAIN_F/` + scripts MATLAB de diseño asociados. NO toca `Moderno/MAIN_F/` (PID baseline) ni `Clasico/`.

**Restricciones invariantes**:
- Ts = 0.1s (no cambiar).
- Zona muerta motor: PWM < 30 prohibido.
- Shutdown térmico: temp > 250°C o thermistor() == -999.
- Forma companion existente: motor 11 estados (W=5, 2W+1), hotend 6 estados (después de `minreal()` desde 201).

---

## Phase 0 — Documentation Discovery (DONE)

### Hallazgos consolidados

**MATLAB — `Moderno/MATLAB/Motor/design_motor_lqr.m`**:
- Carga `state_space_motor.mat` con `A` (11×11), `B` (11×1), `C` (1×11), `D` (1×1), `Ts`.
- Q = `C' * C * 500`, R = 1.
- `[K_motor, ~, ~] = dlqr(A, B, Q, R);` (línea 47).
- `Nbar_motor = rscale_discrete(A, B, C, D, K_motor);` (línea 67).
- Función `rscale_discrete(A,B,C,D,K)` (líneas 113–135): resuelve `M = [A-I, B; C, D]; N = M\[0;1]; Nbar = Nu + K*Nx`.
- **NO guarda a .mat** — variables quedan en workspace para Simulink.

**MATLAB — `Moderno/NN/Hotend/fred_lqr_hotend_design.m`**:
- Carga `state_space_hotend.mat` (201 estados).
- `sys_r = minreal(sys_full, 1e-6); [Ar, Br, Cr, Dr] = ssdata(sys_r);` → 6 estados.
- Q = `Cr' * Cr * 100`, R = 1.
- `[K, ~, ~] = dlqr(Ar, Br, Q, R);` (línea 50).
- `Nbar = rscale_discrete(Ar, Br, Cr, Dr, K);` (línea 66).

**Firmware actual — `LQR_MOTORDC.h`**:
- `K_LQR_M[11] = {9.0774, ...}`, `Nbar_M = 8.39`.
- `rpm_history[11]` shift register.
- `LQRMotor()` aplica `u = Nbar_M*setpoint − Σ K_LQR_M[i]*rpm_history[i]`, satura a `[10, 255]` (BUG: debería ser 30).
- En `LQR_FIJO_MAIN.ino` línea 117, al apagar motor: `rpm_history[i] = 0` (inconsistente con hotend que rellena con `temp_actual`).

**Firmware actual — `LQR_HOTEND.h`**:
- `K_LQR_H[6]`, `Nbar_H = 5.5129`.
- `temp_history[6]` shift register.
- Satura `[0, 255]`. Reset al apagar rellena con `temp_actual` (correcto).

**Protocolo serial actual** (en `LQR_FIJO_MAIN.ino` líneas 154–177):
- `"LQRH:k0,k1,k2,k3,k4,k5,nbar\n"` — 7 floats.
- `"LQRM:k0,...,k10,nbar\n"` — 12 floats.
- GUI LQRMAIN_F **NO** envía LQR (solo PIDH/PIDM); solo `Moderno/Adaptativo/AlFrED0_GUI_V2.py` línea 669 envía LQRH.

### APIs permitidas (no inventar)
- MATLAB: `dlqr`, `ss`, `ssdata`, `minreal`, `lsim`, `blkdiag`. **No** usar `lqi()` (Control Toolbox feature distinta — implementamos servo-LQR manualmente).
- Arduino: `constrain`, `analogRead`, `analogWrite`, `String.startsWith`, `String.indexOf`, `String.substring`, `String.toDouble`. Sin librerías nuevas.

### Anti-patterns a evitar
- ❌ NO usar `lqi()` de MATLAB (asume tiempo continuo y devuelve estructura distinta).
- ❌ NO cambiar el formato `companion form` de los .mat — son input.
- ❌ NO romper el worker adaptativo existente: el parser nuevo debe ser **backward-compatible** (autodetectar N+1 vs N+2 valores).
- ❌ NO agregar librerías Arduino — todo en C++ stdlib.
- ❌ NO tocar `Moderno/MAIN_F/` ni `OG/MAIN_F/`.

---

## Phase 1 — MATLAB: Aumentar a LQI (motor + hotend)

**Archivos a modificar**:
1. `Moderno/MATLAB/Motor/design_motor_lqr.m`
2. `Moderno/NN/Hotend/fred_lqr_hotend_design.m`

### Tareas

**1.1 Motor (`design_motor_lqr.m`)**:
- **Antes** (línea 47): `[K_motor, ~, ~] = dlqr(A, B, Q, R);`
- **Después**: aumentar sistema con integrador del error y rediseñar.

```matlab
% Aumentar sistema para servo-LQR (LQI)
n = size(A, 1);  % 11
A_aug = [A,   zeros(n,1);
         -C,  1];                    % 12×12 (integrador: z(k+1) = z(k) - C*x(k))
B_aug = [B; 0];                       % 12×1
C_aug = [C, 0];                       % 1×12
D_aug = D;                            % 1×1

% Pesos: mantener Q original, agregar peso al integrador
Q_aug = blkdiag(C'*C*500, 50);        % peso integrador = 50 (afinable)
R_aug = 1;

[K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R_aug);
K_motor  = K_aug(1:n);                % 1×11 — realimentación de estados
Ki_motor = K_aug(n+1);                % escalar — ganancia del integrador

% Nbar para servo-LQR (igual fórmula con sistema aumentado, pero ojo:
% en LQI el Nbar suele ser cero porque el integrador hace tracking).
% Mantenemos rscale_discrete para warm-start del integrador.
Nbar_motor = rscale_discrete(A, B, C, D, K_motor);

fprintf('K_motor (1×11): '); disp(K_motor);
fprintf('Ki_motor: %.6f\n', Ki_motor);
fprintf('Nbar_motor: %.6f\n', Nbar_motor);
```

- Imprimir las **12 ganancias en formato Arduino-ready** (copy-paste a `LQR_MOTORDC.h`):
```matlab
fprintf('\n--- COPIAR A LQR_MOTORDC.h ---\n');
fprintf('double K_LQR_M[%d] = {%s};\n', n, ...
    strjoin(arrayfun(@(x) sprintf('%.6f', x), K_motor, 'UniformOutput', false), ', '));
fprintf('double Ki_LQR_M = %.6f;\n', Ki_motor);
fprintf('double Nbar_M = %.6f;\n', Nbar_motor);
```

**1.2 Hotend (`fred_lqr_hotend_design.m`)**:
- Igual estructura, pero después de `minreal()` y con peso integrador conservador (térmica es lenta — empezar con peso = 5).
- **Antes** (línea 50): `[K, ~, ~] = dlqr(Ar, Br, Q, R);`
- **Después**:

```matlab
nr = size(Ar, 1);  % 6 después de minreal
A_aug = [Ar,    zeros(nr,1);
         -Cr,   1];                   % 7×7
B_aug = [Br; 0];
C_aug = [Cr, 0];
D_aug = Dr;

Q_aug = blkdiag(Cr'*Cr*100, 5);       % peso integrador = 5 (térmica lenta)
R_aug = 1;

[K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R_aug);
K  = K_aug(1:nr);                     % 1×6
Ki = K_aug(nr+1);                     % escalar

Nbar = rscale_discrete(Ar, Br, Cr, Dr, K);

fprintf('K_hotend (1×%d): ', nr); disp(K);
fprintf('Ki_hotend: %.6f\n', Ki);
fprintf('Nbar_hotend: %.6f\n', Nbar);
```

### Verificación Phase 1
- [ ] Correr `design_motor_lqr.m` — debe imprimir K_motor (11 valores), Ki_motor, Nbar_motor sin warnings.
- [ ] Correr `fred_lqr_hotend_design.m` — debe imprimir K (6 valores), Ki, Nbar.
- [ ] Simular lazo cerrado con escalón: error en estado estacionario debe ser **0** (vs. el actual no-integral que tiene error).
- [ ] Eigenvalores de `(A_aug − B_aug*K_aug)` dentro del círculo unitario (estable discreto).
- [ ] Si peso integrador genera oscilación: bajarlo (motor 50→20, hotend 5→2).

### Anti-pattern guards Phase 1
- ❌ NO confundir `lqi()` (continuo) con servo-LQR discreto manual.
- ❌ NO olvidar que `rscale_discrete()` se aplica al sistema **original** (A, B, C, D), no al aumentado, ya que Nbar es feedforward del setpoint a la planta.

---

## Phase 2 — Firmware Arduino: Acción Integral + Bug Fixes

**Archivos a modificar**:
1. `Moderno/LQRMAIN_F/LQR_MOTORDC.h`
2. `Moderno/LQRMAIN_F/LQR_HOTEND.h`
3. `Moderno/LQRMAIN_F/LQR_FIJO_MAIN.ino`

### Tareas

**2.1 `LQR_MOTORDC.h` — agregar integrador + anti-windup + zona muerta correcta**:

Agregar variables globales:
```cpp
double Ki_LQR_M = <valor de Phase 1>;   // ganancia integral del LQI
double integral_error_M = 0.0;          // acumulador del error
const double INT_WINDUP_LIMIT_M = 100.0; // límite anti-windup (afinable)
```

Reescribir `LQRMotor()`:
```cpp
double LQRMotor(double rpm_actual, double setpoint) {
    // 1. Shift register
    for (int i = N_ESTADOS_M - 1; i > 0; i--) {
        rpm_history[i] = rpm_history[i-1];
    }
    rpm_history[0] = rpm_actual;

    // 2. Realimentación de estados
    double u_ff = Nbar_M * setpoint;
    double u_fb = 0;
    for (int i = 0; i < N_ESTADOS_M; i++) {
        u_fb += K_LQR_M[i] * rpm_history[i];
    }

    // 3. Acumular integral del error (solo si no saturado — anti-windup condicional)
    double error = setpoint - rpm_actual;
    double u_pre = u_ff - u_fb - Ki_LQR_M * integral_error_M;

    // Anti-windup: solo integrar si la salida no está saturada en la dirección del error
    bool saturated_high = (u_pre >= 255) && (error > 0);
    bool saturated_low  = (u_pre <= 30)  && (error < 0);
    if (!saturated_high && !saturated_low) {
        integral_error_M += error * 0.1;  // Ts = 0.1s
        // Clamp adicional
        if (integral_error_M >  INT_WINDUP_LIMIT_M) integral_error_M =  INT_WINDUP_LIMIT_M;
        if (integral_error_M < -INT_WINDUP_LIMIT_M) integral_error_M = -INT_WINDUP_LIMIT_M;
    }

    double control_u = u_ff - u_fb - Ki_LQR_M * integral_error_M;

    // 4. Setpoint = 0 → apagar
    if (setpoint <= 0) {
        integral_error_M = 0;  // reset integrador
        return 0;
    }

    // 5. Saturación con ZONA MUERTA correcta
    return constrain((int)control_u, 30, 255);   // ← antes era 10
}
```

**2.2 `LQR_HOTEND.h` — mismas adiciones**:

```cpp
double Ki_LQR_H = <valor de Phase 1>;
double integral_error_H = 0.0;
const double INT_WINDUP_LIMIT_H = 50.0;
```

```cpp
double LQRHotend(float temp_actual, double setpoint) {
    for (int i = 5; i > 0; i--) {
        temp_history[i] = temp_history[i-1];
    }
    temp_history[0] = temp_actual;

    double u_ff = Nbar_H * setpoint;
    double u_fb = 0;
    for (int i = 0; i < 6; i++) {
        u_fb += K_LQR_H[i] * temp_history[i];
    }

    double error = setpoint - temp_actual;
    double u_pre = u_ff - u_fb - Ki_LQR_H * integral_error_H;

    bool saturated_high = (u_pre >= 255) && (error > 0);
    bool saturated_low  = (u_pre <= 0)   && (error < 0);
    if (!saturated_high && !saturated_low) {
        integral_error_H += error * 0.1;
        if (integral_error_H >  INT_WINDUP_LIMIT_H) integral_error_H =  INT_WINDUP_LIMIT_H;
        if (integral_error_H < -INT_WINDUP_LIMIT_H) integral_error_H = -INT_WINDUP_LIMIT_H;
    }

    double control_u = u_ff - u_fb - Ki_LQR_H * integral_error_H;
    return constrain(control_u, 0, 255);
}
```

**2.3 `LQR_FIJO_MAIN.ino` — arreglar reset de historial motor + reset integradores al apagar**:

En el bloque de motor apagado (línea ~117):
```cpp
} else {
    analogWrite(pinMotor, 0);
    moto_m = 0;
    // Antes: for(int i=0; i<N_ESTADOS_M; i++) rpm_history[i] = 0;
    // Después: rellenar con rpm_actual (consistente con hotend)
    for (int i = 0; i < N_ESTADOS_M; i++) rpm_history[i] = N_rpm;
    integral_error_M = 0.0;  // reset integrador al apagar
}
```

En el bloque de hotend apagado (línea ~104), agregar reset integrador:
```cpp
} else {
    analogWrite(pinHotend, 0);
    heater_m = 0;
    for(int i=0; i<6; i++) temp_history[i] = temp_actual;
    integral_error_H = 0.0;  // reset integrador al apagar
}
```

### Verificación Phase 2
- [ ] Compilar `LQR_FIJO_MAIN.ino` en Arduino IDE / arduino-cli (Mega 2560) sin errores.
- [ ] Grep `constrain.*10.*255` en `LQR_MOTORDC.h` debe devolver 0 hits (ya cambiado a 30).
- [ ] Grep `rpm_history\[i\] = 0` en `LQR_FIJO_MAIN.ino` debe devolver 0 hits.
- [ ] Verificar que `integral_error_M` y `integral_error_H` aparezcan declaradas en sus respectivos `.h`.

### Anti-pattern guards Phase 2
- ❌ NO usar `int` para `integral_error` — debe ser `double` para precisión.
- ❌ NO olvidar `* 0.1` (Ts) en el acumulador integral.
- ❌ NO integrar cuando saturado en dirección del error (clásico windup).
- ❌ NO bajar de PWM < 30 en motor (zona muerta del hardware).

---

## Phase 3 — Protocolo serial backward-compatible

**Archivo a modificar**: `Moderno/LQRMAIN_F/LQR_FIJO_MAIN.ino` (parser LQRH/LQRM en `processInput`).

### Tarea

Extender el parser para autodetectar formato viejo vs nuevo:
- **Formato viejo** (worker adaptativo actual): `"LQRH:k0..k5,nbar\n"` (7 floats), `"LQRM:k0..k10,nbar\n"` (12 floats).
- **Formato nuevo** (con LQI): `"LQRH:k0..k5,ki,nbar\n"` (8 floats), `"LQRM:k0..k10,ki,nbar\n"` (13 floats).

Detección: contar comas en el payload. Si comas == N_ESTADOS, es viejo. Si comas == N_ESTADOS+1, es nuevo (incluye Ki).

Reemplazar handler `LQRM:` (líneas ~168–177):

```cpp
else if (command.startsWith("LQRM:")) {
    String vals = command.substring(5);
    // Contar comas
    int n_commas = 0;
    for (int i = 0; i < (int)vals.length(); i++) if (vals[i] == ',') n_commas++;

    int pos = 0;
    for (int i = 0; i < N_ESTADOS_M; i++) {
        int comma = vals.indexOf(',', pos);
        if (comma < 0) break;
        K_LQR_M[i] = vals.substring(pos, comma).toDouble();
        pos = comma + 1;
    }
    if (n_commas == N_ESTADOS_M + 1) {
        // Formato nuevo: incluye Ki_LQR
        int comma = vals.indexOf(',', pos);
        Ki_LQR_M = vals.substring(pos, comma).toDouble();
        pos = comma + 1;
        Nbar_M = vals.substring(pos).toDouble();
        Serial.println("LQR_M actualizado (con Ki)");
    } else {
        // Formato viejo: solo Nbar
        Nbar_M = vals.substring(pos).toDouble();
        Serial.println("LQR_M actualizado (legacy, sin Ki)");
    }
    integral_error_M = 0.0;  // reset al recibir nuevos gains
}
```

Mismo patrón para `LQRH:` (con `6` en vez de `N_ESTADOS_M`).

### Verificación Phase 3
- [ ] Test con `"LQRM:1,2,3,4,5,6,7,8,9,10,11,99.9\n"` (12 floats viejo) → imprime "legacy, sin Ki", `Ki_LQR_M` no cambia.
- [ ] Test con `"LQRM:1,2,3,4,5,6,7,8,9,10,11,0.5,99.9\n"` (13 floats nuevo) → imprime "con Ki", `Ki_LQR_M = 0.5`, `Nbar_M = 99.9`.
- [ ] Worker adaptativo existente (`Moderno/Adaptativo/Adaptativo_Worker.py`) sigue funcionando sin cambios.

### Anti-pattern guards Phase 3
- ❌ NO romper el worker existente — autodetect, no requerir flag.
- ❌ NO contar comas con regex (no soportado en Arduino String).

---

## Phase 4 — Verificación end-to-end

### Tareas

**4.1 Compilación y firmware**:
- [ ] `arduino-cli compile --fqbn arduino:avr:mega Moderno/LQRMAIN_F/` exitoso (o IDE).
- [ ] Flashear a Mega 2560.

**4.2 Smoke test simulación MATLAB**:
- [ ] Correr ambos design scripts. Verificar polos cerrados estables.
- [ ] Simular lazo cerrado con setpoint paso. Confirmar **error estacionario = 0** (era ≠0 antes).

**4.3 Smoke test físico** (con FrED real, opcional si hardware disponible):
- [ ] Setpoint motor = 20 RPM → debe llegar exactamente a 20 RPM (antes ~18-22).
- [ ] Setpoint hotend = 190°C → debe llegar exactamente a 190 (antes ~185-195).
- [ ] Verificar no oscila (si oscila, bajar peso integrador en MATLAB y re-correr Phase 1).

**4.4 Documentación**:
- [ ] Actualizar `Moderno/LQRMAIN_F/README-LQR-FIJO.md` con: nueva ley de control (`u = Nbar·r − K·x − Ki·∫error`), valores nuevos de K/Ki/Nbar, nota sobre zona muerta corregida.

### Verificación final con greps
- [ ] Grep `"constrain.*10.*255"` en `LQR_MOTORDC.h` → 0 hits.
- [ ] Grep `"rpm_history\[i\] = 0"` en `LQR_FIJO_MAIN.ino` → 0 hits.
- [ ] Grep `"Ki_LQR"` en `LQR_MOTORDC.h` y `LQR_HOTEND.h` → ≥1 hit cada uno.
- [ ] Grep `"integral_error"` en `LQR_FIJO_MAIN.ino` → ≥2 hits (reset motor + reset hotend).

---

## Resumen de archivos modificados

| Archivo | Phase | Cambio |
|---|---|---|
| `Moderno/MATLAB/Motor/design_motor_lqr.m` | 1 | Aumentar a sistema 12-est, calcular Ki_motor |
| `Moderno/NN/Hotend/fred_lqr_hotend_design.m` | 1 | Aumentar a sistema 7-est, calcular Ki_hotend |
| `Moderno/LQRMAIN_F/LQR_MOTORDC.h` | 2 | Ki_LQR_M, integral_error_M, anti-windup, PWM mín 30 |
| `Moderno/LQRMAIN_F/LQR_HOTEND.h` | 2 | Ki_LQR_H, integral_error_H, anti-windup |
| `Moderno/LQRMAIN_F/LQR_FIJO_MAIN.ino` | 2+3 | Reset integradores, fix reset historia motor, parser dual-format |
| `Moderno/LQRMAIN_F/README-LQR-FIJO.md` | 4 | Doc actualizada |

## Decisiones explícitas

1. **Servo-LQR manual, no `lqi()` de MATLAB** — porque trabajamos en discreto y queremos control total sobre la augmentación.
2. **Anti-windup condicional** (clamp + integrar solo si no saturado) — más simple que back-calculation y suficiente para esta aplicación.
3. **Backward-compatible** en parser serial — preserva el worker adaptativo existente sin tocarlo.
4. **No tocar GUI Python** — el LQR FIJO no requiere UI nueva. Si se quiere LQI adaptativo después, es trabajo separado.
5. **Pesos integrador iniciales**: motor=50, hotend=5 — afinables en Phase 1 si hay oscilación.
