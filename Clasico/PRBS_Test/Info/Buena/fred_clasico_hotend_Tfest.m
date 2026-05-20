% FrED Control Clásico — Identificación Hotend
% Step responses: PWM 60, 100, 180, 220
% G(s) = K / (tau*s + 1) * e^(-theta*s)
clear; clc; close all;

cd 'C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\PRBS_Test\Info\Buena'

%% ============================================================
%  1. CARGAR
%% ============================================================
h60  = readtable('3)hotend60.csv');
h100 = readtable('1)hotend100.csv');
h180 = readtable('2)hotend180.csv');
h220 = readtable('4)hotend220.csv');

n_zeros = 50;
[y60,  u60]  = prep_step(h60.temp_C,  h60.pwm_heater,  n_zeros);
[y100, u100] = prep_step(h100.temp_C, h100.pwm_heater, n_zeros);
[y180, u180] = prep_step(h180.temp_C, h180.pwm_heater, n_zeros);
[y220, u220] = prep_step(h220.temp_C, h220.pwm_heater, n_zeros);

dh60  = iddata(y60,  u60,  0.1);
dh100 = iddata(y100, u100, 0.1);
dh180 = iddata(y180, u180, 0.1);
dh220 = iddata(y220, u220, 0.1);

fprintf('Muestras — PWM60: %d | PWM100: %d | PWM180: %d | PWM220: %d\n', ...
    length(y60)-n_zeros, length(y100)-n_zeros, length(y180)-n_zeros, length(y220)-n_zeros);

fprintf('\n=== Curva estática del hotend ===\n');
fprintf('  PWM  60 → SS = %.1f°C\n', h60.temp_C(find(~isnan(h60.temp_C), 1, 'last')));
fprintf('  PWM 100 → SS = %.1f°C\n', h100.temp_C(end));
fprintf('  PWM 180 → SS = %.1f°C\n', h180.temp_C(end));
fprintf('  PWM 220 → SS = %.1f°C\n', h220.temp_C(end));

%% ============================================================
%  2. IDENTIFICACIÓN
%% ============================================================
fprintf('\n=== Identificación con merge(h100, h180) ===\n');

d_train  = merge(dh100, dh180);
d_val60  = dh60;
d_val220 = dh220;

opt = tfestOptions('EnforceStability', true);

sys_h1  = tfest(d_train, 1, 0, opt);
sys_h1d = tfest(d_train, 1, 0, opt, 'InputDelay', 1.0);
sys_h2  = tfest(d_train, 2, 0, opt);

[~, fit_h1_60]   = compare(d_val60,  sys_h1);
[~, fit_h1_220]  = compare(d_val220, sys_h1);
[~, fit_h1d_60]  = compare(d_val60,  sys_h1d);
[~, fit_h1d_220] = compare(d_val220, sys_h1d);
[~, fit_h2_60]   = compare(d_val60,  sys_h2);
[~, fit_h2_220]  = compare(d_val220, sys_h2);

fprintf('\n  FIT en PWM=60 (val bajo):\n');
fprintf('    1er orden:           %.1f%%\n', fit_h1_60);
fprintf('    1er orden + retardo: %.1f%%\n', fit_h1d_60);
fprintf('    2do orden:           %.1f%%\n', fit_h2_60);

fprintf('\n  FIT en PWM=220 (val alto):\n');
fprintf('    1er orden:           %.1f%%\n', fit_h1_220);
fprintf('    1er orden + retardo: %.1f%%\n', fit_h1d_220);
fprintf('    2do orden:           %.1f%%\n', fit_h2_220);

%% ============================================================
%  3. PARÁMETROS FÍSICOS
%% ============================================================
fprintf('\n=== G(s) Hotend ===\n');
sys_h1
sys_h1d
sys_h2

K_h  = dcgain(sys_h1);
K_h2 = dcgain(sys_h2);
fprintf('Ganancia estática K (1er orden) = %.4f °C/PWM\n', K_h);
fprintf('Ganancia estática K (2do orden) = %.4f °C/PWM\n', K_h2);

%% ============================================================
%  4. RESIDUOS
%% ============================================================
figure('Name','Residuos hotend 1er orden');
resid(d_val60, sys_h1);
title('Residuos G(s) hotend 1er orden — val PWM=60');

figure('Name','Residuos hotend 1er orden + retardo');
resid(d_val60, sys_h1d);
title('Residuos G(s) hotend 1er orden + retardo — val PWM=60');

%% ============================================================
%  5. GRÁFICAS
%% ============================================================
figure('Name','Comparación modelos hotend');
compare(d_val60, sys_h1, sys_h1d, sys_h2);
title('G(s) Hotend — validación PWM=60');
legend('1er orden','1er orden + retardo','2do orden');

figure('Name','Comparación hotend PWM=220');
compare(d_val220, sys_h1, sys_h1d, sys_h2);
title('G(s) Hotend — validación PWM=220');
legend('1er orden','1er orden + retardo','2do orden');

figure('Name','Respuesta escalón hotend');
step(sys_h1 * 100, sys_h1d * 100);
title('Respuesta escalón Hotend (entrada PWM=100)');
ylabel('Delta Temp (°C)'); xlabel('Tiempo (s)'); grid on;
legend('1er orden','1er orden + retardo');

%% ============================================================
%  6. DISCRETIZAR Y GUARDAR
%% ============================================================
sys_hd1  = c2d(tf(sys_h1),  0.1, 'zoh');
sys_hd1d = c2d(tf(sys_h1d), 0.1, 'zoh');
sys_hd2  = c2d(tf(sys_h2),  0.1, 'zoh');

fprintf('\nG(z) hotend 1er orden (ZOH, Ts=0.1s):\n');
sys_hd1

save('hotend_Gs.mat', 'sys_h1', 'sys_h1d', 'sys_h2', 'sys_hd1', 'sys_hd1d', 'sys_hd2');
fprintf('\nGuardado: hotend_Gs.mat\n');
fprintf('Siguiente: fred_clasico_pid_design.m\n');

%% ============================================================
%  FUNCIÓN LOCAL
%% ============================================================
function [y_clean, u_clean] = prep_step(temp, pwm, n_zeros)
    valid   = ~isnan(temp) & ~isnan(pwm);
    temp    = temp(valid);
    pwm     = pwm(valid);
    y_clean = temp - temp(1);
    y_clean = [zeros(n_zeros, 1); y_clean];
    u_clean = [zeros(n_zeros, 1); pwm];
end