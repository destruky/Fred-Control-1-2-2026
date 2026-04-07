% =========================================================================
% PREPARACIÓN DEL WORKSPACE PARA SIMULINK - HOTEND
% =========================================================================
clc; clear; close all;

ruta_script = fileparts(mfilename('fullpath'));

% 1. Cargar el modelo térmico identificado
% lqr_valores_hotend.mat fue generado en Simulink aplicando minreal()
% al state_space_hotend.mat (201 estados, generado por hotend_sysid_nn.py)
% con tolerancia 1e-6. Resultado: ~11 estados controlables.
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

disp('-----------------------------------------');
disp('¡Variables reducidas del Hotend listas para Simulink!');
disp('Ya puedes darle "Run" a tu modelo .slx');

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