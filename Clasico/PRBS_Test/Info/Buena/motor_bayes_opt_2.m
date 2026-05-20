clear; clc; close all;

%% Ruta base del script
base = fileparts(mfilename('fullpath'));

%% Cargar modelo base
load(fullfile(base, 'fred_modelos_pid.mat'));

Tsim = 25;   % Tiempo de simulación (motor rápido: 20-30 s)

%% Variables a optimizar
vars = [
    optimizableVariable('Kp', [0.1, 10])
    optimizableVariable('Ki', [0.01, 10])
    optimizableVariable('Kd', [0.0, 10])
];

%% Función objetivo (ITAE)
objFcn = @(x) motorITAE(x, G_motor, Tsim);

%% Optimización Bayesiana
results = bayesopt(objFcn, vars, ...
    'MaxObjectiveEvaluations', 25, ...
    'IsObjectiveDeterministic', true, ...
    'AcquisitionFunctionName', 'expected-improvement-plus');

best = results.XAtMinObjective;

Kp_opt = best.Kp;
Ki_opt = best.Ki;
Kd_opt = best.Kd;

%% Guardar resultados
save(fullfile(base, 'pid_motor_bayes.mat'), ...
    'Kp_opt', 'Ki_opt', 'Kd_opt');

fprintf('\n=== PID Optimizado Motor ===\n');
fprintf('Kp = %.4f\n', Kp_opt);
fprintf('Ki = %.4f\n', Ki_opt);
fprintf('Kd = %.4f\n', Kd_opt);

%% Comparar contra baseline
C_actual = pid(25, 2.5, 1.5);
sys_actual = feedback(C_actual * G_motor, 1);

t_base = linspace(0, Tsim, 1000);
[y_base, t_base] = step(sys_actual, t_base);

e_base = 1 - y_base;
ITAE_actual = trapz(t_base, t_base .* abs(e_base));

fprintf('\n=== Comparativa ITAE ===\n');
fprintf('PID actual FrED (Kp=25, Ki=2.5, Kd=1.5): ITAE = %.4f\n', ITAE_actual);
fprintf('PID optimizado BO:                         ITAE = %.4f\n', results.MinObjective);
fprintf('Mejora: %.1f%%\n', ...
    (ITAE_actual - results.MinObjective) / ITAE_actual * 100);

%% Graficar respuesta final optimizada
C_opt = pid(Kp_opt, Ki_opt, Kd_opt);
sys_opt = feedback(C_opt * G_motor, 1);

figure
step(sys_opt, Tsim)
grid on
title('Respuesta al escalón - PID Optimizado Motor')
xlabel('Tiempo (s)')
ylabel('Salida')

%% Función objetivo ITAE
function J = motorITAE(x, G_motor, Tsim)

    C = pid(x.Kp, x.Ki, x.Kd);
    sys_cl = feedback(C * G_motor, 1);

    % Verificación de estabilidad
    if any(real(pole(sys_cl)) >= 0)
        J = 1e6;
        return
    end

    t = linspace(0, Tsim, 1000);
    [y, t] = step(sys_cl, t);

    e = 1 - y;
    J = trapz(t, t .* abs(e));
end