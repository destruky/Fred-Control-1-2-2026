% pidtuner_baseline.m
% Calcula ganancias PID baseline con pidtune() sobre G(s) identificadas.
% Estas ganancias entran a Tabla 2/3/4 (Simulink) y Tabla 5/6 (hardware) como columna "PID Tuner".

clear; clc; close all;

% Cargar modelos identificados (G_motor y G_hotend)
load('C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\PRBS_Test\Info\Buena\fred_modelos_pid.mat');

% ============================================================
% MOTOR DC — pidtune sobre G_motor
% ============================================================
fprintf('=================================================\n');
fprintf('MOTOR DC — pidtune()\n');
fprintf('=================================================\n');

% Usar el modelo con retardo de transporte: el retardo aporta fase y obliga
% a pidtune a usar accion proporcional (evita el Kp=0 degenerado).
S_mot = load('C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\PRBS_Test\Info\Buena\motor_Gs.mat');
C_motor = pidtune(S_mot.sys_tf1d, 'PI', 2);  % 1er orden + retardo, PI, wc=2
Kp_M = C_motor.Kp;
Ki_M = C_motor.Ki;
Kd_M = C_motor.Kd;

fprintf('Kp = %.4f\n', Kp_M);
fprintf('Ki = %.4f\n', Ki_M);
fprintf('Kd = %.4f\n', Kd_M);
fprintf('PIDM:%.4f,%.4f,%.4f   <-- comando para el GUI\n\n', Kp_M, Ki_M, Kd_M);

% Métricas en simulación (setpoint 20 RPM, 30 s)
sys_cl_M = feedback(C_motor * G_motor, 1);
t_M = linspace(0, 30, 1000);
y_M = step(20 * sys_cl_M, t_M);
e_M = 20 - y_M;
ITAE_M = trapz(t_M, t_M .* abs(e_M));
info_M = stepinfo(y_M, t_M, 20);

fprintf('--- Métricas Motor (setpoint 20 RPM, 30 s) ---\n');
fprintf('ITAE             = %.4f\n', ITAE_M);
fprintf('T. establecimiento = %.2f s\n', info_M.SettlingTime);
fprintf('Sobretiro         = %.2f %%\n', info_M.Overshoot);
fprintf('Error estacionario = %.4f RPM\n\n', abs(20 - y_M(end)));

% ============================================================
% HOTEND — pidtune sobre G_hotend
% ============================================================
fprintf('=================================================\n');
fprintf('HOTEND — pidtune()\n');
fprintf('=================================================\n');

C_hotend = pidtune(G_hotend, 'PID', 0.25);  % wc=0.25 rad/s → Kp comparable al manual
Kp_H = C_hotend.Kp;
Ki_H = C_hotend.Ki;
Kd_H = C_hotend.Kd;

fprintf('Kp = %.4f\n', Kp_H);
fprintf('Ki = %.4f\n', Ki_H);
fprintf('Kd = %.4f\n', Kd_H);
fprintf('PIDH:%.4f,%.4f,%.4f   <-- comando para el GUI\n\n', Kp_H, Ki_H, Kd_H);

% Métricas en simulación (setpoint 200 °C, 800 s)
sys_cl_H = feedback(C_hotend * G_hotend, 1);
t_H = linspace(0, 800, 2000);
y_H = step(200 * sys_cl_H, t_H);
e_H = 200 - y_H;
ITAE_H = trapz(t_H, t_H .* abs(e_H));
info_H = stepinfo(y_H, t_H, 200);

fprintf('--- Métricas Hotend (setpoint 200 °C, 800 s) ---\n');
fprintf('ITAE             = %.4f\n', ITAE_H);
fprintf('T. establecimiento = %.2f s\n', info_H.SettlingTime);
fprintf('Sobretiro         = %.2f %%\n', info_H.Overshoot);
fprintf('Error estacionario = %.4f °C\n\n', abs(200 - y_H(end)));

% ============================================================
% Guardar ganancias para uso posterior
% ============================================================
save('pidtuner_gains.mat', 'Kp_M', 'Ki_M', 'Kd_M', 'Kp_H', 'Ki_H', 'Kd_H');
fprintf('Ganancias guardadas en pidtuner_gains.mat\n\n');

% ============================================================
% Gráficas comparativas (escalón simulado)
% ============================================================
% ============================================================
% Gráficas
% ============================================================

figure;
plot(t_M, y_M, 'b-', 'LineWidth', 2); hold on;
yline(20, 'k--', 'Referencia 20 RPM');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Motor DC - PID Tuner'); grid on;

figure;
plot(t_H, y_H, 'r-', 'LineWidth', 2); hold on;
yline(200, 'k--', 'Referencia 200 C');
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title('Hotend - PID Tuner'); grid on;
