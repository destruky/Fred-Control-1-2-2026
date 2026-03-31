% FrED Control Clásico — Validación G(s) + PID desde modelo
clear; clc; close all;

cd 'C:\Users\rquin\OneDrive\Desktop\Fred-Testing\FrED-TEC\Clasico\PRBS_Test\Info\Buena'

load('motor_Gs.mat')
load('hotend_Gs.mat')

%% ============================================================
%  1. VALIDAR G(s) MOTOR con lsim()
%% ============================================================
m2  = readtable('PRBS_Motor2.csv');
t_m = m2.t_ms / 1000;
u_m = double(m2.pwm);
y_m = double(m2.rpm);

y_sim_motor = lsim(tf(sys_tf2), u_m, t_m);

rmse_motor = sqrt(mean((y_sim_motor - y_m).^2));
r2_motor   = 1 - sum((y_m - y_sim_motor).^2) / sum((y_m - mean(y_m)).^2);

fprintf('=== Validación G(s) Motor ===\n');
fprintf('  RMSE = %.2f RPM\n', rmse_motor);
fprintf('  R²   = %.4f\n', r2_motor);

figure('Name','Validación Motor DC');
idx = t_m <= 30;
plot(t_m(idx), y_m(idx), 'r-', 'LineWidth', 1, 'DisplayName', 'Real'); hold on;
plot(t_m(idx), y_sim_motor(idx), 'b--', 'LineWidth', 1.5, 'DisplayName', 'G(s) simulado');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Validación G(s) Motor DC — zoom 0-30s');
legend; grid on;

%% ============================================================
%  2. VALIDAR G(s) HOTEND con lsim()
%% ============================================================
h60   = readtable('3)hotend60.csv');
valid = ~isnan(h60.temp_C);
t_h   = h60.t_ms(valid) / 1000;
u_h   = double(h60.pwm_heater(valid));
y_h   = double(h60.temp_C(valid));

y_sim_hotend = lsim(tf(sys_h1), u_h, t_h) + y_h(1);

rmse_hotend = sqrt(mean((y_sim_hotend - y_h).^2));
r2_hotend   = 1 - sum((y_h - y_sim_hotend).^2) / sum((y_h - mean(y_h)).^2);

fprintf('\n=== Validación G(s) Hotend ===\n');
fprintf('  RMSE = %.2f °C\n', rmse_hotend);
fprintf('  R²   = %.4f\n', r2_hotend);

figure('Name','Validación Hotend');
plot(t_h, y_h, 'r-', 'LineWidth', 0.8, 'DisplayName', 'Real'); hold on;
plot(t_h, y_sim_hotend, 'b--', 'LineWidth', 1.2, 'DisplayName', 'G(s) simulado');
xlabel('Tiempo (s)'); ylabel('Temperatura (°C)');
title('Validación G(s) Hotend — datos reales vs simulado (PWM=60)');
legend; grid on;

%% ============================================================
%  3. PIDTUNE — Motor
%% ============================================================
fprintf('\n=== PID Motor (pidtune) ===\n');
C_motor_tune = pidtune(tf(sys_tf2), 'PID');
[Kp_m, Ki_m, Kd_m] = piddata(C_motor_tune);
fprintf('  Kp = %.4f\n', Kp_m);
fprintf('  Ki = %.4f\n', Ki_m);
fprintf('  Kd = %.4f\n', Kd_m);
fprintf('  Actuales en FrED: Kp=25, Ki=2.5, Kd=1.5\n');

%% ============================================================
%  4. PIDTUNE — Hotend
%% ============================================================
fprintf('\n=== PID Hotend (pidtune) ===\n');
C_hotend_tune = pidtune(tf(sys_h1), 'PID');
[Kp_h, Ki_h, Kd_h] = piddata(C_hotend_tune);
fprintf('  Kp = %.4f\n', Kp_h);
fprintf('  Ki = %.4f\n', Ki_h);
fprintf('  Kd = %.4f\n', Kd_h);
fprintf('  Actuales en FrED: Kp=1.8, Ki=0.9, Kd=0.3\n');

%% ============================================================
%  5. GUARDAR para Simulink
%% ============================================================
G_motor  = tf(sys_tf2);
G_hotend = tf(sys_h1);

save('fred_modelos_pid.mat', ...
    'G_motor',  'G_hotend', ...
    'C_motor_tune',  'C_hotend_tune', ...
    'Kp_m', 'Ki_m', 'Kd_m', ...
    'Kp_h', 'Ki_h', 'Kd_h');

fprintf('\nGuardado: fred_modelos_pid.mat\n');
fprintf('Siguiente: Simulink con G_motor y G_hotend\n');