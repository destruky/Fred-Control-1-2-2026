% 1. Carga las matrices que hiciste en Python
% Este archivo debe existir en tu carpeta actual
load('state_space_motor.mat'); 

% 2. Parámetros del motor (basados en tus datos de FrED-TEC)
Ts = 0.1; % 100ms

% 3. Diseño LQR
% Q: Peso del error (35 RPM). R: Peso del esfuerzo (PWM).
Q = C' * C * 500; 
R = 1; 

% 4. AQUÍ SE CREA LA VARIABLE QUE TE FALTA
[K_motor, S, e] = dlqr(A, B, Q, R);

% 5. Cálculo de Nbar (Pre-compensación)
Nbar_motor = 1 / (C * inv(eye(size(A)) - (A - B*K_motor)) * B);

disp('Variable K_motor creada con éxito en el Workspace.');