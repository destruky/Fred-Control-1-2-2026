% =========================================================================
% CONTROL LQR — HOTEND FrED (Control Moderno)
% =========================================================================
% Planta: red neuronal PyTorch linealizada → A,B,C,D (11 estados)
% Problema: rank(controlabilidad) = 6/11 — dlqr falla en sistema completo
% Solución: minreal() reduce al subespacio controlable+observable (6 est.)
% =========================================================================

clc; clear; close all;

% -------------------------------------------------------------------------
% 1. RUTAS — dinámicas, sin hardcodear paths
% -------------------------------------------------------------------------
ruta_script = fileparts(mfilename('fullpath'));
mat_file    = fullfile(ruta_script, '..', 'NN', 'state_space_hotend.mat');

if ~exist(mat_file, 'file')
    % Fallback: buscar en la misma carpeta del script
    mat_file = fullfile(ruta_script, 'state_space_hotend.mat');
end
if ~exist(mat_file, 'file')
    error('No se encontró state_space_hotend.mat\nBusqué en: %s', mat_file);
end

load(mat_file);
Ts_val = Ts(1,1);  % extraer escalar del array 1x1

% -------------------------------------------------------------------------
% 2. SISTEMA COMPLETO (11 estados) — solo para diagnóstico
% -------------------------------------------------------------------------
sys_full = ss(A, B, C, D, Ts_val);

fprintf('Sistema completo: %d estados\n', size(A,1));
fprintf('  |eigenvalues|: %s\n', mat2str(abs(eig(A))', 4));

Wc = ctrb(A, B);
fprintf('  Rank controlabilidad: %d/%d\n', rank(Wc), size(A,1));

% -------------------------------------------------------------------------
% 3. REDUCCIÓN CON minreal() — extrae subespacio controlable+observable
% -------------------------------------------------------------------------
sys_r = minreal(sys_full, 1e-6);
[Ar, Br, Cr, Dr] = ssdata(sys_r);
nr = size(Ar, 1);

fprintf('\nSistema reducido (minreal): %d estados\n', nr);
fprintf('  |eigenvalues|: %s\n', mat2str(abs(eig(Ar))', 4));

% Verificar que el sistema reducido es controlable
if rank(ctrb(Ar, Br)) < nr
    error('Sistema reducido sigue siendo incontrolable — revisar state_space_hotend.mat');
end

% -------------------------------------------------------------------------
% 4. DISEÑO LQR SOBRE SISTEMA REDUCIDO
% -------------------------------------------------------------------------
% Q penaliza error de temperatura | R penaliza esfuerzo de control (PWM)
% Subir Q → respuesta más rápida pero más overshooot
% Subir R → PWM más conservador pero más lento
Q = Cr' * Cr * 100;
R = 1;

[K, ~, ~] = dlqr(Ar, Br, Q, R);

eigs_cl = abs(eig(Ar - Br*K));
fprintf('\nGanancia K (1x%d):\n', nr);
disp(K);
fprintf('|eigenvalues| lazo cerrado: %s\n', mat2str(eigs_cl', 4));

if all(eigs_cl < 1)
    fprintf('Lazo cerrado: ESTABLE ✓\n');
else
    warning('Lazo cerrado INESTABLE — ajustar Q o R');
end

% -------------------------------------------------------------------------
% 5. PREALIMENTACIÓN Nbar (discreta)
% -------------------------------------------------------------------------
% Condición steady-state discreta: x(k+1) = x(k) → (A-I)x + Bu = 0
% Se resuelve: [A-I, B; C, D] \ [0; 1], luego Nbar = Nu + K*Nx
Nbar = rscale_discrete(Ar, Br, Cr, Dr, K);
fprintf('Nbar = %.6f\n', Nbar);

% -------------------------------------------------------------------------
% 6. SIMULACIÓN — lazo cerrado con prealimentación, ref=190°C, 300s
% -------------------------------------------------------------------------
sys_cl = ss(Ar - Br*K, Br*Nbar, Cr, Dr, Ts_val);

t   = (0:Ts_val:300)';
ref = 190 * ones(size(t));

[y_sim, t_sim] = lsim(sys_cl, ref, t);

% -------------------------------------------------------------------------
% 7. GRÁFICA
% -------------------------------------------------------------------------
figure('Name', 'LQR Hotend — FrED', 'Color', 'w');
plot(t_sim, y_sim, 'b', 'LineWidth', 2); hold on;
yline(190, 'r--', 'LineWidth', 1.5);
title('Respuesta Térmica Hotend — Control LQR');
xlabel('Tiempo (s)');
ylabel('Temperatura (°C)');
legend('LQR simulado', 'Ref 190°C', 'Location', 'southeast');
grid on;
set(gca, 'FontSize', 12);

% Métricas básicas
[~, idx_90] = min(abs(y_sim - 0.9*190));
fprintf('\nTiempo de subida (10%%→90%%): ~%.1f s\n', t_sim(idx_90));
ss_err = abs(y_sim(end) - 190);
fprintf('Error estado estacionario: %.4f\n', ss_err);

% =========================================================================
% FUNCIÓN: rscale_discrete
% Prealimentación correcta para sistemas discretos.
%
% En tiempo discreto, estado estacionario → x(k+1) = x(k):
%   (A - I)*x_ss + B*u_ss = 0
%   C*x_ss + D*u_ss = 1
% → Resuelve el sistema 2D para Nx, Nu, luego Nbar = Nu + K*Nx
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
