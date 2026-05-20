clear; clc; close all;
load('fred_modelos_pid.mat');

% PID BO Motor
Kp = 2.3480; Ki = 9.9974; Kd = 0.2064;
C = pid(Kp, Ki, Kd);
sys_cl = feedback(C * G_motor, 1);
t = linspace(0, 25, 1000);
[y, t] = step(30 * sys_cl, t);

% Métricas
e = 30 - y;
ITAE = trapz(t, t .* abs(e));
info = stepinfo(y, t, 30);
fprintf('ITAE = %.4f\n', ITAE);
fprintf('Settling time = %.2f s\n', info.SettlingTime);
fprintf('Overshoot = %.2f %%\n', info.Overshoot);
fprintf('Ess = %.4f RPM\n', abs(30 - y(end)));

figure;
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(30, 'k--', 'Referencia 30 RPM');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Motor DC — PID BO'); grid on;