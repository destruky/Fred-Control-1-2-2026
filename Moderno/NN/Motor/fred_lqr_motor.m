% =========================================================================
% CONTROL LQR — MOTOR DC FrED (Control Moderno)
% =========================================================================
% Planta: red neuronal PyTorch linealizada → A,B,C,D
% Instrucciones: Setpoint 30 RPM, 30 segundos, Q = C'*C*100, R = 1
% =========================================================================

clc; clear; close all;

% -------------------------------------------------------------------------
% 1. CARGAR DATOS
% -------------------------------------------------------------------------
% Busca el archivo state_space_motor.mat en la misma carpeta
if ~exist('state_space_motor.mat', 'file')
    error('No se encontró state_space_motor.mat en esta carpeta.');
end

load('state_space_motor.mat');
Ts_val = Ts(1,1);

% -------------------------------------------------------------------------
% 2. SISTEMA Y CONTROLABILIDAD
% -------------------------------------------------------------------------
sys_full = ss(A, B, C, D, Ts_val);
n_estados = size(A,1);

fprintf('Sistema Motor: %d estados\n', n_estados);
fprintf('  Rank controlabilidad: %d/%d\n', rank(ctrb(A,B)), n_estados);

% -------------------------------------------------------------------------
% 3. DISEÑO LQR
% -------------------------------------------------------------------------
Q = C' * C * 100;
R = 1;

[K, ~, ~] = dlqr(A, B, Q, R);

eigs_cl = abs(eig(A - B*K));
fprintf('\nGanancia K (1x%d):\n', n_estados);
disp(K);

if all(eigs_cl < 1)
    fprintf('Lazo cerrado: ESTABLE ✓\n');
else
    warning('Lazo cerrado INESTABLE');
end

% -------------------------------------------------------------------------
% 4. PREALIMENTACIÓN Nbar (discreta)
% -------------------------------------------------------------------------
Nbar = rscale_discrete(A, B, C, D, K);
fprintf('Nbar = %.6f\n', Nbar);

% -------------------------------------------------------------------------
% 5. SIMULACIÓN — 30 RPM por 30s
% -------------------------------------------------------------------------
sys_cl = ss(A - B*K, B*Nbar, C, D, Ts_val);

t   = (0:Ts_val:30)';
ref = 30 * ones(size(t));

[y_sim, t_sim] = lsim(sys_cl, ref, t);

% -------------------------------------------------------------------------
% 6. GRÁFICA
% -------------------------------------------------------------------------
figure('Name', 'LQR Motor — FrED', 'Color', 'w');
plot(t_sim, y_sim, 'b', 'LineWidth', 2); hold on;
yline(30, 'r--', 'LineWidth', 1.5);
title('Respuesta Motor DC — Control LQR');
xlabel('Tiempo (s)');
ylabel('Velocidad (RPM)');
legend('LQR simulado', 'Ref 30 RPM', 'Location', 'southeast');
grid on;
set(gca, 'FontSize', 12);

% =========================================================================
% FUNCIÓN: rscale_discrete
% =========================================================================
function Nbar = rscale_discrete(A, B, C, D, K)
    n   = size(A, 1);
    M   = [A - eye(n), B; C, D];
    rhs = [zeros(n, 1); 1];

    if rank(M) < size(M, 1)
        warning('rscale: sistema singular, usando pseudoinversa');
        N = pinv(M) * rhs;
    else
        N = M \ rhs;
    end

    Nx   = N(1:n);
    Nu   = N(n+1);
    Nbar = Nu + K * Nx;
end