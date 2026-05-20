% FrED Control Clásico — Identificación Motor DC (1 base de datos)
% PRBS re-capturado con el mecanismo extrusor (zona muerta PWM<120).
% Validación honesta: split train/validación 70/30 sobre el mismo dataset.
clear; clc; close all;

cd 'C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\PRBS_Test\Info\Buena'

%% ============================================================
%  1. CARGAR Y PREPROCESAR
%% ============================================================
csv_path = 'C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\PRBSmotorfinal.csv';
m1 = readtable(csv_path);
fprintf('PRBSmotorfinal: %d muestras (%.1f s @ Ts=0.1s)\n', ...
    height(m1), height(m1)*0.1);

%% ============================================================
%  VENTANA LIMPIA — saltar arranque en frío, evitar deriva térmica
%  El dataset completo (33 min) NO es time-invariant. Se usa una
%  ventana de 500s en régimen térmico estable para identificar.
%% ============================================================
IDX_INI = 300;           % saltar primeros 30s (motor pre-calentado)
IDX_FIN = height(m1);    % usar todo el resto del dataset
mw = m1(IDX_INI:IDX_FIN, :);
fprintf('Ventana de identificación: muestras %d-%d (%.0f-%.0f s)\n', ...
    IDX_INI, IDX_FIN, IDX_INI*0.1, IDX_FIN*0.1);

% Detrend — remover media (tfest identifica dinámica, no el offset)
u = detrend(mw.pwm, 0);
y = detrend(mw.rpm, 0);
d_all = iddata(y, u, 0.1);

%% ============================================================
%  2. SPLIT TRAIN / VALIDACIÓN (70 / 30) dentro de la ventana
%% ============================================================
N   = numel(d_all.y);
Ntr = round(0.70 * N);
d_train = d_all(1:Ntr);
d_val   = d_all(Ntr+1:end);
fprintf('Train: %d muestras | Validación: %d muestras\n\n', Ntr, N-Ntr);

%% ============================================================
%  3. IDENTIFICAR — 1er orden, 2do orden, 1er orden + retardo
%% ============================================================
fprintf('=== Identificación (tfest sobre train) ===\n');

sys_tf1  = tfest(d_train, 1, 0);
sys_tf2  = tfest(d_train, 2, 0);
sys_tf1d = tfest(d_train, 1, 0, 'InputDelay', 0.1);

[~, fit1]  = compare(d_val, sys_tf1);
[~, fit2]  = compare(d_val, sys_tf2);
[~, fit1d] = compare(d_val, sys_tf1d);

fprintf('  1er orden:           %.1f%% (validación)\n', fit1);
fprintf('  2do orden:           %.1f%% (validación)\n', fit2);
fprintf('  1er orden + retardo: %.1f%% (validación)\n', fit1d);

%% ============================================================
%  4. SELECCIÓN AUTOMÁTICA — mejor FIT en validación
%% ============================================================
[fit_best, idx] = max([fit1, fit2, fit1d]);
modelos  = {sys_tf1, sys_tf2, sys_tf1d};
nombres  = {'1er orden', '2do orden', '1er orden + retardo'};
G_motor  = tf(modelos{idx});
fprintf('\n>> Modelo seleccionado: %s (FIT=%.1f%%)\n', nombres{idx}, fit_best);

%% ============================================================
%  5. PARÁMETROS FÍSICOS
%% ============================================================
K1 = dcgain(sys_tf1);
K2 = dcgain(sys_tf2);
fprintf('\n=== Parámetros físicos ===\n');
fprintf('G(s) 1er orden:\n'); sys_tf1
fprintf('G(s) 2do orden:\n'); sys_tf2
fprintf('K (1er orden) = %.4f RPM/PWM\n', K1);
fprintf('K (2do orden) = %.4f RPM/PWM\n', K2);
fprintf('RPM@PWM200 (1er) = %.1f\n', K1*200);
fprintf('RPM@PWM200 (2do) = %.1f\n', K2*200);

%% ============================================================
%  6. ANÁLISIS DE RESIDUOS — modelo seleccionado
%% ============================================================
figure('Name','Residuos modelo seleccionado');
resid(d_val, G_motor);
title(sprintf('Residuos G(s) %s — validación', nombres{idx}));

%% ============================================================
%  7. GRÁFICAS
%% ============================================================
figure('Name','Comparación modelos');
compare(d_val, sys_tf1, sys_tf2, sys_tf1d);
title('G(s) Motor DC — validación (30% holdout)');
legend('1er orden','2do orden','1er orden + retardo');

figure('Name','Respuesta escalón');
step(sys_tf1*200, sys_tf2*200);
title('Respuesta escalón Motor DC (PWM=200)');
ylabel('RPM'); xlabel('Tiempo (s)'); grid on;
legend('1er orden','2do orden');

%% ============================================================
%  8. DISCRETIZAR Y GUARDAR
%% ============================================================
sys_d1 = c2d(tf(sys_tf1), 0.1, 'zoh');
sys_d2 = c2d(tf(sys_tf2), 0.1, 'zoh');

fprintf('\nG(z) modelo seleccionado (ZOH, Ts=0.1s):\n');
c2d(G_motor, 0.1, 'zoh')

% motor_Gs.mat — todos los modelos
save('motor_Gs.mat', 'sys_tf1', 'sys_tf2', 'sys_tf1d', 'sys_d1', 'sys_d2', 'G_motor');
fprintf('\nGuardado: motor_Gs.mat\n');

% fred_modelos_pid.mat — actualizar G_motor preservando G_hotend
mat_pid = 'fred_modelos_pid.mat';
if isfile(mat_pid)
    S = load(mat_pid);
else
    S = struct();
end
S.G_motor = G_motor;
save(mat_pid, '-struct', 'S');
fprintf('Actualizado: fred_modelos_pid.mat (G_motor reemplazado, G_hotend preservado)\n');
