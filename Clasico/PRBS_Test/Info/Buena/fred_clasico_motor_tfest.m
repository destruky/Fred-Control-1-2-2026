% FrED Control Clásico — Identificación Motor DC (versión correcta)
% Usa merge() en lugar de concatenación + validación cruzada honesta
clear; clc; close all;

cd 'C:\Users\rquin\OneDrive\Desktop\Fred-Testing\FrED-TEC\Clasico\PRBS_Test\Info\Buena'

%% ============================================================
%  1. CARGAR Y PREPROCESAR
%% ============================================================
m1 = readtable('PRBS_Motor1.csv');
m2 = readtable('PRBS_Motor2.csv');
m3 = readtable('PRBS_Motor3.csv');

% Detrend — remover media
d1 = iddata(detrend(m1.rpm,0), detrend(m1.pwm,0), 0.1);
d2 = iddata(detrend(m2.rpm,0), detrend(m2.pwm,0), 0.1);
d3 = iddata(detrend(m3.rpm, 0), detrend(m3.pwm, 0), 0.1);
fprintf('Motor1: %d | Motor2: %d | Motor3: %d muestras\n', ...
    height(m1), height(m2), height(m3));

%% ============================================================
%  2. VALIDACIÓN CRUZADA — 3 FOLDS
%% ============================================================
fprintf('\n=== Validación cruzada (3 folds) ===\n');

folds = {
    merge(d1,d2), d3, 'train{1,2} val{3}';
    merge(d1,d3), d2, 'train{1,3} val{2}';
    merge(d2,d3), d1, 'train{2,3} val{1}';
};

fits_tf1 = zeros(3,1);
fits_tf2 = zeros(3,1);

for i = 1:3
    tr  = folds{i,1};
    v   = folds{i,2};
    tag = folds{i,3};

    s1 = tfest(tr, 1, 0);
    s2 = tfest(tr, 2, 0);

    [~, f1] = compare(v, s1);
    [~, f2] = compare(v, s2);

    fits_tf1(i) = f1;
    fits_tf2(i) = f2;

    fprintf('  Fold %d (%s):\n', i, tag);
    fprintf('    1er orden: %.1f%%   2do orden: %.1f%%\n', f1, f2);
end

fprintf('\n  Promedio 1er orden: %.1f%%\n', mean(fits_tf1));
fprintf('  Promedio 2do orden: %.1f%%\n', mean(fits_tf2));
fprintf('  Std     1er orden: %.1f%%\n', std(fits_tf1));
fprintf('  Std     2do orden: %.1f%%\n', std(fits_tf2));

%% ============================================================
%  3. MODELO FINAL — train con Motor1+Motor2, val Motor3
%% ============================================================
fprintf('\n=== Modelo final (train Motor1+Motor2, val Motor3) ===\n');

d_train_final = merge(d1, d2);
d_val_final   = d3;

sys_tf1  = tfest(d_train_final, 1, 0);
sys_tf2  = tfest(d_train_final, 2, 0);
sys_tf1d = tfest(d_train_final, 1, 0, 'InputDelay', 0.1);

[~, fit1]  = compare(d_val_final, sys_tf1);
[~, fit2]  = compare(d_val_final, sys_tf2);
[~, fit1d] = compare(d_val_final, sys_tf1d);

fprintf('  1er orden:           %.1f%%\n', fit1);
fprintf('  2do orden:           %.1f%%\n', fit2);
fprintf('  1er orden + retardo: %.1f%%\n', fit1d);

%% ============================================================
%  4. PARÁMETROS FÍSICOS
%% ============================================================
K1 = dcgain(sys_tf1);
K2 = dcgain(sys_tf2);
fprintf('\n=== Parámetros físicos ===\n');
fprintf('G(s) 1er orden:\n'); sys_tf1
fprintf('G(s) 2do orden:\n'); sys_tf2
fprintf('K (1er orden) = %.4f RPM/PWM\n', K1);
fprintf('K (2do orden) = %.4f RPM/PWM\n', K2);
fprintf('RPM@PWM90 (1er) = %.1f\n', K1*90);
fprintf('RPM@PWM90 (2do) = %.1f\n', K2*90);

%% ============================================================
%  5. ANÁLISIS DE RESIDUOS — modelo seleccionado
%% ============================================================
figure('Name','Residuos 1er orden');
resid(d_val_final, sys_tf1);
title('Residuos G(s) 1er orden — Motor3');

figure('Name','Residuos 2do orden');
resid(d_val_final, sys_tf2);
title('Residuos G(s) 2do orden — Motor3');

%% ============================================================
%  6. GRÁFICAS
%% ============================================================
figure('Name','Comparación modelos');
compare(d_val_final, sys_tf1, sys_tf2, sys_tf1d);
title('G(s) Motor DC — validación en Motor3');
legend('1er orden','2do orden','1er orden + retardo');

figure('Name','Respuesta escalón');
step(sys_tf1*90, sys_tf2*90);
title('Respuesta escalón Motor DC (PWM=90)');
ylabel('RPM'); xlabel('Tiempo (s)'); grid on;
legend('1er orden','2do orden');

%% ============================================================
%  7. DISCRETIZAR Y GUARDAR
%% ============================================================
sys_d1 = c2d(tf(sys_tf1), 0.1, 'zoh');
sys_d2 = c2d(tf(sys_tf2), 0.1, 'zoh');

fprintf('\nG(z) 2do orden (ZOH, Ts=0.1s):\n');
sys_d2

save('motor_Gs.mat', 'sys_tf1', 'sys_tf2', 'sys_tf1d', 'sys_d1', 'sys_d2');
fprintf('\nGuardado: motor_Gs.mat\n');
fprintf('Siguiente: fred_clasico_hotend_Gs.m\n');