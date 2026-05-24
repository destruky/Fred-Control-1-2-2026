% =========================================================================
% PREPARACIÓN DEL WORKSPACE PARA SIMULINK - HOTEND
% ROL: WORKSPACE SIMULINK — prepara variables para Hotendcontrol.slx.
%      Requiere lqr_valores_hotend.mat (generado desde Hotendcontrol.slx, NO desde
%      fred_lqr_hotend_design.m). Carga Ar,Br,Cr,Dr, aplica minreal() y diseña LQR.
% =========================================================================
clc; clear; close all;

ruta_script = fileparts(mfilename('fullpath'));

% 1. Cargar el modelo térmico identificado
% lqr_valores_hotend.mat fue generado en Simulink aplicando minreal()
% al state_space_hotend.mat (201 estados, generado por hotend_sysid_nn.py)
% con tolerancia 1e-6. Resultado: ~6 estados controlables.
% El flujo de reducción está en Hotendcontrol.slx — no es reproducible desde script.
% Variables contenidas: Ar, Br, Cr, Dr (sistema reducido en espacio de estados)
load(fullfile(ruta_script, 'lqr_valores_hotend.mat'));

% 2. Extraer el tiempo de muestreo
Ts_val = 0.1; 

% 3. Crear sistema completo (11 estados)
sys_full = ss(Ar, Br, Cr, Dr, Ts_val);

% 4. REDUCCIÓN (Crucial para que funcione el LQR en Simulink)
% Reduce a los 6 estados que realmente importan y asigna a las variables de Simulink
sys_r = minreal(sys_full, 1e-6);
[A_hotend, B_hotend, C_hotend, D_hotend] = ssdata(sys_r);

% 5. Diseño LQR sobre el sistema reducido
Q = C_hotend' * C_hotend * 100;
R = 1;

[K_hotend, ~, ~] = dlqr(A_hotend, B_hotend, Q, R);

% 6. Calcular ganancia de Pre-compensación (Nbar) usando tu función
Nbar_hotend = rscale_discrete(A_hotend, B_hotend, C_hotend, D_hotend, K_hotend);

% --- NUEVA SECCIÓN AGREGADA PARA LQI (SERVO-LQR) ---
nr = size(A_hotend, 1);
A_aug = [A_hotend, zeros(nr,1); -C_hotend, 1];
B_aug = [B_hotend; 0];
Q_aug = blkdiag(C_hotend'*C_hotend*100, 5); % Peso integrador = 5
[K_aug, ~, ~] = dlqr(A_aug, B_aug, Q_aug, R);
K_LQI = K_aug(1:nr);
Ki_LQI = K_aug(nr+1);

fprintf('\n--- COPIAR A LQR_HOTEND.h ---\n');
fprintf('double K_LQR_H[%d] = {%s};\n', nr, strjoin(arrayfun(@(x) sprintf('%.6f', x), K_LQI, 'UniformOutput', false), ', '));
fprintf('double Ki_LQR_H = %.6f;\n', Ki_LQI);
fprintf('double Nbar_H = %.6f;\n', Nbar_hotend);
fprintf('--------------------------------\n');

disp('¡Variables reducidas del Hotend listas para Simulink!');
disp('Ya puedes darle "Run" a tu modelo .slx');

% --- AGREGADO: SIMULACIÓN Y GRÁFICA DE VALIDACIÓN ---
sys_cl = ss(A_hotend - B_hotend*K_LQI, B_hotend*Nbar_hotend, C_hotend, D_hotend, Ts_val);
t_sim = (0:Ts_val:800)'; 
setpoint = 190 * ones(size(t_sim));
[y_sim, ~] = lsim(sys_cl, setpoint, t_sim);

figure('Name', 'LQR Hotend — FrED', 'Color', 'w');
plot(t_sim, y_sim, 'r', 'LineWidth', 2); hold on;
yline(190, 'k--', 'LineWidth', 1.5);
title('Comportamiento esperado: Respuesta Hotend (Servo-LQI)');
xlabel('Tiempo (s)');
ylabel('Temperatura (°C)');
legend('LQR simulado', 'Ref 190 °C', 'Location', 'southeast');
grid on; set(gca, 'FontSize', 12);

% =========================================================================
% FUNCIÓN: rscale_discrete (Debe ir al final del script)
% =========================================================================
function Nbar = rscale_discrete(A, B, C, D, K)
    n   = size(A, 1);
    M   = [A - eye(n), B; C, D];
    rhs = [zeros(n, 1); 1];
    if rank(M) < size(M, 1)
        N = pinv(M) * rhs;
    else
        N = M \ rhs;
    end
    Nx   = N(1:n);
    Nu   = N(n+1);
    Nbar = Nu + K * Nx;
end