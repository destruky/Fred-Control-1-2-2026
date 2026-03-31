% =========================================================================
% CONTROL LQR — HOTEND FrED (Control Moderno)
% =========================================================================
% Planta: NN NARX linealizada → A,B,C,D (201 estados, companion form)
% Incluye estados de temp(k..k-W) Y pwm(k-1..k-W) para capturar
% correctamente la ganancia DC del hotend.
%
% Con 201 estados no se puede implementar directo en Arduino.
% Solución: reducción de modelo con balred() → ~5-10 estados → dlqr.
% =========================================================================

clc; clear; close all;

% -------------------------------------------------------------------------
% 1. RUTAS
% -------------------------------------------------------------------------
ruta_script = fileparts(mfilename('fullpath'));
mat_file    = fullfile(ruta_script, 'NN', 'Hotend', 'state_space_hotend.mat');

if ~exist(mat_file, 'file')
    mat_file = fullfile(ruta_script, 'state_space_hotend.mat');
end
if ~exist(mat_file, 'file')
    error('No se encontro state_space_hotend.mat\nBusque en: %s', mat_file);
end

load(mat_file);
Ts_val = Ts(1,1);

% -------------------------------------------------------------------------
% 2. DIAGNOSTICO DEL SISTEMA COMPLETO
% -------------------------------------------------------------------------
n_full = size(A, 1);
sys_full = ss(A, B, C, D, Ts_val);

fprintf('Sistema completo: %d estados, Ts=%.2fs\n', n_full, Ts_val);
fprintf('  |eigenvalues| max: %.4f\n', max(abs(eig(A))));
fprintf('  Rank controlabilidad: %d/%d\n', rank(ctrb(A, B)), n_full);

% DC gain del sistema completo
dc_full = dcgain(sys_full);
fprintf('  DC gain: %.4f C/PWM (empirica ≈ 0.9-1.1)\n', dc_full);

% -------------------------------------------------------------------------
% 3. REDUCCION DE MODELO
% -------------------------------------------------------------------------
% balred(): reducción balanceada que preserva la respuesta I/O
% Probamos con distintos órdenes y elegimos el menor que mantenga
% DC gain y respuesta al escalón similares.

fprintf('\nReduccion de modelo (balred):\n');

best_n = 0;
best_sys = [];
for n_red = [3, 5, 7, 10, 15]
    try
        sys_r = balred(sys_full, n_red);
        dc_r  = dcgain(sys_r);
        dc_err = abs(dc_r - dc_full) / abs(dc_full) * 100;

        fprintf('  n=%2d: DC=%.4f (error=%.1f%%)', n_red, dc_r, dc_err);

        if dc_err < 5 && dc_r > 0
            fprintf(' <-- OK\n');
            if best_n == 0
                best_n = n_red;
                best_sys = sys_r;
            end
        else
            fprintf('\n');
        end
    catch ME
        fprintf('  n=%2d: ERROR %s\n', n_red, ME.message);
    end
end

if best_n == 0
    % Fallback: usar minreal
    fprintf('\nbalred fallo, intentando minreal...\n');
    best_sys = minreal(sys_full, 1e-6);
    [Ar, Br, Cr, Dr] = ssdata(best_sys);
    best_n = size(Ar, 1);
    fprintf('  minreal: %d estados\n', best_n);
end

[Ar, Br, Cr, Dr] = ssdata(best_sys);
fprintf('\nSistema reducido: %d estados\n', best_n);
fprintf('  DC gain: %.4f C/PWM\n', dcgain(best_sys));
fprintf('  Rank controlabilidad: %d/%d\n', rank(ctrb(Ar, Br)), best_n);

% Verificar que el sistema reducido sea controlable
if rank(ctrb(Ar, Br)) < best_n
    warning('Sistema reducido no completamente controlable');
end

% -------------------------------------------------------------------------
% 4. DISENO LQR SOBRE SISTEMA REDUCIDO
% -------------------------------------------------------------------------
Q = Cr' * Cr * 100;
R = 1;

[K, ~, ~] = dlqr(Ar, Br, Q, R);

eigs_cl = abs(eig(Ar - Br*K));
fprintf('\nGanancia K (1x%d):\n', best_n);
disp(K);
fprintf('|eigenvalues| lazo cerrado: %s\n', mat2str(eigs_cl', 4));

if all(eigs_cl < 1)
    fprintf('Lazo cerrado: ESTABLE\n');
else
    warning('Lazo cerrado INESTABLE — ajustar Q o R');
end

% -------------------------------------------------------------------------
% 5. PREALIMENTACION Nbar (discreta)
% -------------------------------------------------------------------------
nr = size(Ar, 1);
M   = [Ar - eye(nr), Br; Cr, Dr];
rhs = [zeros(nr, 1); 1];
N   = M \ rhs;
Nx  = N(1:nr);
Nu  = N(nr+1);
Nbar = Nu + K * Nx;
fprintf('Nbar = %.6f\n', Nbar);

% -------------------------------------------------------------------------
% 6. SIMULACION — ref=190C, 300s
% -------------------------------------------------------------------------
sys_cl = ss(Ar - Br*K, Br*Nbar, Cr, Dr, Ts_val);

t   = (0:Ts_val:300)';
ref = 190 * ones(size(t));

[y_sim, t_sim] = lsim(sys_cl, ref, t);

% Metricas
info = stepinfo(y_sim, t_sim, 190);
fprintf('\n--- Metricas de respuesta ---\n');
fprintf('  Tiempo de subida (10-90%%): %.1f s\n', info.RiseTime);
fprintf('  Settling time (2%%):        %.1f s\n', info.SettlingTime);
fprintf('  Overshoot:                  %.2f%%\n', info.Overshoot);
fprintf('  Error estado estacionario:  %.4f C\n', abs(y_sim(end) - 190));

% -------------------------------------------------------------------------
% 7. GRAFICA
% -------------------------------------------------------------------------
figure('Name', 'LQR Hotend — FrED', 'Color', 'w');

subplot(2,1,1);
plot(t_sim, y_sim, 'b', 'LineWidth', 2); hold on;
yline(190, 'r--', 'LineWidth', 1.5);
title(sprintf('LQR Hotend (sistema reducido %d estados)', best_n));
xlabel('Tiempo (s)');
ylabel('Temperatura (C)');
legend('LQR simulado', 'Ref 190 C', 'Location', 'southeast');
grid on; set(gca, 'FontSize', 12);

% Comparar step response del sistema completo vs reducido
subplot(2,1,2);
[y_full, t_full] = step(sys_full, 300);
[y_red,  t_red]  = step(best_sys, 300);
plot(t_full, y_full, 'b-', 'LineWidth', 1.5); hold on;
plot(t_red, y_red, 'r--', 'LineWidth', 1.5);
title('Validacion: step completo vs reducido');
xlabel('Tiempo (s)');
ylabel('Temperatura (C)');
legend(sprintf('Completo (%d est.)', n_full), ...
       sprintf('Reducido (%d est.)', best_n), 'Location', 'southeast');
grid on; set(gca, 'FontSize', 12);

fprintf('\n--- Valores para Arduino ---\n');
fprintf('K = [');
fprintf('%.6f ', K);
fprintf(']\n');
fprintf('Nbar = %.6f\n', Nbar);
fprintf('Ts = %.2f s\n', Ts_val);
fprintf('Estados del sistema reducido: %d\n', best_n);
