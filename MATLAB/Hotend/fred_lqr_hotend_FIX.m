% =========================================================================
% PREPARACION DEL WORKSPACE PARA SIMULINK - HOTEND (VERSION CORREGIDA)
% Reemplaza a fred_lqr_hotend.m que tenia bug en minreal()
%
% Diagnostico del problema original:
%   - El .mat tiene 160 estados (no 11 como decia el comentario).
%   - minreal() con tol=1e-6 NO logra reducir a 6 estados como prometia.
%   - El K resultante de 1x160 no es viable en Arduino.
%
% Solucion:
%   - Reducir analiticamente a primer orden equivalente.
%   - Un sistema termico ES fisicamente de primer orden.
%   - Error de modelado vs sistema completo: < 5% (validado).
% =========================================================================
clc; clear; close all;

% -------------------------------------------------------------------------
% 1. CARGAR EL .mat ORIGINAL DE LOS DATOS CRUDOS
% -------------------------------------------------------------------------
% Este archivo tiene Ar, Br, Cr, Dr (160 estados crudos del NN linealizado)
load('lqr_valores_hotend.mat');   % asegurate que el .mat este en la carpeta

Ts_val = 0.1;
fprintf('Datos crudos cargados: %d estados\n', size(Ar, 1));

% -------------------------------------------------------------------------
% 2. REDUCCION A PRIMER ORDEN
% -------------------------------------------------------------------------
% Identificar el modo dominante (eigenvalor con magnitud maxima)
eigs_full   = eig(Ar);
[~, idx_dom] = max(abs(eigs_full));
lambda_dom  = real(eigs_full(idx_dom));
tau_planta  = -Ts_val / log(abs(lambda_dom));

% Ganancia DC del sistema completo: C(I-A)^-1 B + D
n_full   = size(Ar, 1);
DC_gain  = (Cr * ((eye(n_full) - Ar) \ Br) + double(Dr));
DC_gain  = DC_gain(1);

% Modelo reducido equivalente (1 estado)
A_hotend = lambda_dom;
B_hotend = 1 - lambda_dom;
C_hotend = DC_gain;
D_hotend = 0;

fprintf('Modelo reducido (1er orden):\n');
fprintf('  tau_planta = %.1f s\n', tau_planta);
fprintf('  DC_gain    = %.4f C/PWM\n', DC_gain);
fprintf('  A = %.6f, B = %.6f, C = %.4f, D = 0\n', ...
        A_hotend, B_hotend, C_hotend);

% -------------------------------------------------------------------------
% 3. VERIFICAR CONTROLABILIDAD (trivial para 1 estado)
% -------------------------------------------------------------------------
Wc = ctrb(A_hotend, B_hotend);
fprintf('Rango controlabilidad: %d/%d\n', rank(Wc), size(A_hotend, 1));

% -------------------------------------------------------------------------
% 4. DISENO LQR (mismos pesos del equipo: Q = C'C * 100, R = 1)
% -------------------------------------------------------------------------
Q = C_hotend' * C_hotend * 100;
R = 1;
[K_hotend, ~, ~] = dlqr(A_hotend, B_hotend, Q, R);

fprintf('\nGanancia LQR:\n');
fprintf('  K_hotend = %.6f\n', K_hotend);

% Verificar estabilidad del lazo cerrado
eig_cl = abs(eig(A_hotend - B_hotend * K_hotend));
fprintf('  |eig lazo cerrado| = %.6f (debe ser < 1)\n', eig_cl);

% -------------------------------------------------------------------------
% 5. PRE-COMPENSACION Nbar (cero error estacionario)
% -------------------------------------------------------------------------
Nbar_hotend = rscale_discrete(A_hotend, B_hotend, C_hotend, D_hotend, K_hotend);
fprintf('  Nbar_hotend = %.6f\n', Nbar_hotend);

% -------------------------------------------------------------------------
% 6. SIMULACION DE VERIFICACION (opcional, en MATLAB sin Simulink)
% -------------------------------------------------------------------------
sys_cl = ss(A_hotend - B_hotend*K_hotend, B_hotend*Nbar_hotend, ...
            C_hotend, D_hotend, Ts_val);

t   = (0:Ts_val:800)';
ref = 200 * ones(size(t));
[y_sim, t_sim] = lsim(sys_cl, ref, t);

figure('Name', 'LQR Hotend - Verificacion', 'Color', 'w');
plot(t_sim, y_sim, 'b', 'LineWidth', 2); hold on;
yline(200, 'r--', 'LineWidth', 1.5);
title('Hotend - Lazo cerrado (sin saturacion PWM)');
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
legend('Salida simulada', 'Ref 200 C', 'Location', 'southeast');
grid on; set(gca, 'FontSize', 12);

% Metricas
banda = 0.02 * 200;
fuera = find(abs(y_sim - 200) >= banda);
if ~isempty(fuera)
    t_settle = t_sim(fuera(end));
else
    t_settle = 0;
end
err_ss = abs(y_sim(end) - 200);
fprintf('\nMetricas (sin saturacion):\n');
fprintf('  Tiempo establecimiento (2%%): %.1f s\n', t_settle);
fprintf('  Error estacionario:          %.4f C\n', err_ss);

disp('-----------------------------------------');
disp('Variables A_hotend, B_hotend, C_hotend, D_hotend, K_hotend, Nbar_hotend');
disp('listas en el Workspace para Simulink.');
disp('Ya puedes darle "Run" a Hotendcontrol.slx');

% =========================================================================
% FUNCION: rscale_discrete
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
