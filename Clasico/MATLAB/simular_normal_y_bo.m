% simular_normal_y_bo.m
% Genera las 4 graficas Simulink restantes:
%   - Motor con PID Normal (manual, ano pasado)
%   - Motor con PID BO
%   - Hotend con PID Normal (manual)
%   - Hotend con PID BO

clear; clc; close all;
load('fred_modelos_pid.mat');

% ============================================================
% MOTOR — PID NORMAL (manual)
% ============================================================
Kp = 25;  Ki = 2.5;  Kd = 1.5;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C * G_motor, 1);
t = linspace(0, 30, 1000);
y = step(20 * sys_cl, t);
ITAE = trapz(t, t .* abs(20 - y));
info = stepinfo(y, t, 20);
fprintf('--- Motor PID Normal ---\n');
fprintf('ITAE=%.4f  Ts=%.2fs  OS=%.2f%%  Ess=%.4f RPM\n\n', ...
    ITAE, info.SettlingTime, info.Overshoot, abs(20-y(end)));

figure;
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(20, 'k--', 'Referencia 20 RPM');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Motor DC - PID Normal'); grid on;

% ============================================================
% MOTOR — PID BO
% ============================================================
Kp = 2.3480;  Ki = 9.9974;  Kd = 0.2064;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C * G_motor, 1);
t = linspace(0, 30, 1000);
y = step(20 * sys_cl, t);
ITAE = trapz(t, t .* abs(20 - y));
info = stepinfo(y, t, 20);
fprintf('--- Motor PID BO ---\n');
fprintf('ITAE=%.4f  Ts=%.2fs  OS=%.2f%%  Ess=%.4f RPM\n\n', ...
    ITAE, info.SettlingTime, info.Overshoot, abs(20-y(end)));

figure;
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(20, 'k--', 'Referencia 20 RPM');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Motor DC - PID BO'); grid on;

% ============================================================
% HOTEND — PID NORMAL (manual)
% ============================================================
Kp = 1.8;  Ki = 0.9;  Kd = 0.3;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C * G_hotend, 1);
t = linspace(0, 800, 2000);
y = step(200 * sys_cl, t);
ITAE = trapz(t, t .* abs(200 - y));
info = stepinfo(y, t, 200);
fprintf('--- Hotend PID Normal ---\n');
fprintf('ITAE=%.2e  Ts=%.1fs  OS=%.2f%%  Ess=%.4f C\n\n', ...
    ITAE, info.SettlingTime, info.Overshoot, abs(200-y(end)));

figure;
plot(t, y, 'r-', 'LineWidth', 2); hold on;
yline(200, 'k--', 'Referencia 200 C');
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title('Hotend - PID Normal'); grid on;

% ============================================================
% HOTEND — PID BO
% ============================================================
Kp = 9.9657;  Ki = 4.9947;  Kd = 0.0183;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C * G_hotend, 1);
t = linspace(0, 800, 2000);
y = step(200 * sys_cl, t);
ITAE = trapz(t, t .* abs(200 - y));
info = stepinfo(y, t, 200);
fprintf('--- Hotend PID BO ---\n');
fprintf('ITAE=%.2e  Ts=%.1fs  OS=%.2f%%  Ess=%.4f C\n\n', ...
    ITAE, info.SettlingTime, info.Overshoot, abs(200-y(end)));

figure;
plot(t, y, 'r-', 'LineWidth', 2); hold on;
yline(200, 'k--', 'Referencia 200 C');
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title('Hotend - PID BO'); grid on;
