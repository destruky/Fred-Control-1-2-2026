% =========================================================================
% DISEÑO LQR — MOTOR DC FrED (Control Moderno)
% =========================================================================
% Planta: red neuronal PyTorch linealizada → A,B,C,D (6 estados, Ts=0.1s)
% Origen: state_space_motor.mat generado por motor_sysid_nn.py
%         Punto de operación: PWM=75, RPM=28. Sistema estable en lazo abierto.
% =========================================================================
clc; clear; close all;

% -------------------------------------------------------------------------
% 1. CARGAR MATRICES
% -------------------------------------------------------------------------
ruta_script = fileparts(mfilename('fullpath'));
mat_file    = fullfile(ruta_script, '..', '..', 'NN', 'Motor', 'state_space_motor.mat');

if ~exist(mat_file, 'file')
    mat_file = fullfile(ruta_script, 'state_space_motor.mat');
end
if ~exist(mat_file, 'file')
    error('No se encontró state_space_motor.mat\nBusqué en: %s', mat_file);
end

% state_space_motor.mat fue generado por motor_sysid_nn.py (Moderno/NN/Motor/).
% Contiene A, B, C, D, Ts — companion form completa 2W+1=11 estados.
% Punto de operación: PWM=75, RPM=28. Sistema estable en lazo abierto.
% A diferencia del hotend, el motor no requiere minreal() — todos los estados
% son controlables directamente. El flujo de simulación está en Motorcontrol.slx
% y no es reproducible desde script.
load(mat_file);
Ts_val = Ts(1,1);

fprintf('Sistema cargado: %d estados, Ts=%.2fs\n', size(A,1), Ts_val);
fprintf('  |eigenvalues| lazo abierto: %s\n', mat2str(abs(eig(A))', 4));

% -------------------------------------------------------------------------
% 2. VERIFICAR CONTROLABILIDAD
% -------------------------------------------------------------------------
nr = size(A,1);
Wc = ctrb(A, B);
rk = rank(Wc);
fprintf('  Rank controlabilidad: %d/%d\n', rk, nr);

if rk < nr
    warning('Sistema no completamente controlable — revisar state_space_motor.mat');
end

% -------------------------------------------------------------------------
% 3. DISEÑO LQR
% -------------------------------------------------------------------------
% Q penaliza error de RPM | R penaliza esfuerzo de control (PWM)
% Subir Q → respuesta más rápida pero más sobreimpulso
% Subir R → PWM más conservador pero más lento
Q = C' * C * 500;
R = 1;

[K_motor, ~, ~] = dlqr(A, B, Q, R);

% -------------------------------------------------------------------------
% 4. VERIFICAR ESTABILIDAD DEL LAZO CERRADO
% -------------------------------------------------------------------------
eigs_cl = abs(eig(A - B*K_motor));
fprintf('\nGanancia K_motor (1x%d):\n', nr);
disp(K_motor);
fprintf('|eigenvalues| lazo cerrado: %s\n', mat2str(eigs_cl', 4));

if all(eigs_cl < 1)
    fprintf('Lazo cerrado: ESTABLE ✓\n');
else
    warning('Lazo cerrado INESTABLE — ajustar Q o R');
end

% -------------------------------------------------------------------------
% 5. PREALIMENTACIÓN Nbar (discreta)
% -------------------------------------------------------------------------
Nbar_motor = rscale_discrete(A, B, C, D, K_motor);
fprintf('Nbar_motor = %.6f\n', Nbar_motor);

% -------------------------------------------------------------------------
% 6. SIMULACIÓN — lazo cerrado, ref=28 RPM, 10s
% -------------------------------------------------------------------------
sys_cl = ss(A - B*K_motor, B*Nbar_motor, C, D, Ts_val);

t   = (0:Ts_val:10)';
ref = 28 * ones(size(t));

[y_sim, t_sim] = lsim(sys_cl, ref, t);

% -------------------------------------------------------------------------
% 7. GRÁFICA
% -------------------------------------------------------------------------
figure('Name', 'LQR Motor — FrED', 'Color', 'w');
plot(t_sim, y_sim, 'b', 'LineWidth', 2); hold on;
yline(28, 'r--', 'LineWidth', 1.5);
title('Respuesta Motor DC — Control LQR');
xlabel('Tiempo (s)');
ylabel('RPM');
legend('LQR simulado', 'Ref 28 RPM', 'Location', 'southeast');
grid on;
set(gca, 'FontSize', 12);

% Métricas básicas
[~, idx_90] = min(abs(y_sim - 0.9*28));
fprintf('\nTiempo de subida (0→90%%): ~%.2f s\n', t_sim(idx_90));
ss_err = abs(y_sim(end) - 28);
fprintf('Error estado estacionario: %.4f RPM\n', ss_err);

disp('-----------------------------------------');
disp('Variables K_motor y Nbar_motor listas para Simulink.');
disp('Ya puedes darle "Run" a tu modelo .slx');

% =========================================================================
% FUNCIÓN: rscale_discrete
% Prealimentación correcta para sistemas discretos.
% En tiempo discreto, estado estacionario → x(k+1) = x(k):
%   (A - I)*x_ss + B*u_ss = 0
%   C*x_ss + D*u_ss = 1
% → Resuelve el sistema para Nx, Nu, luego Nbar = Nu + K*Nx
% =========================================================================
function Nbar = rscale_discrete(A, B, C, D, K)
    n   = size(A, 1);
    M   = [A - eye(n), B; C, D];
    rhs = [zeros(n, 1); 1];

    if rank(M) < size(M, 1)
        warning('rscale: sistema singular, usando pseudoinversa (puede ser impreciso)');
        N = pinv(M) * rhs;
    else
        N = M \ rhs;
    end

    Nx   = N(1:n);
    Nu   = N(n+1);
    Nbar = Nu + K * Nx;
end
