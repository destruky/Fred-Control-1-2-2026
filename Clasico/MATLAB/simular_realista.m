    % simular_realista.m
% Simulacion multi-rate del PID con las salvaguardas del firmware:
%   - Control digital a Ts=0.1s (igual que MAIN_F.ino)
%   - Planta evoluciona con Ts_sub=Ts/10=0.01s (integracion fina)
%   - Saturacion PWM en [0, 255]
%   - Anti-windup: integracion condicional con desbloqueo por signo del error
%     (mas robusto que back-calc cuando hay retraso de medicion / filtro encoder)
%   - Derivada sobre la salida medida (no sobre el error)
%   - Motor: zona muerta PWM<30 + filtro paso bajo del encoder (alpha=0.90)
%   - Hotend: precalentado a 150C + escalon a 200C + perturbacion 200->220C en t=400s
%
% Genera 6 figuras Simulink-like en Clasico/Resultados/Figuras/.

clear; clc; close all;
load('C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\PRBS_Test\Info\Buena\fred_modelos_pid.mat');

Ts = 0.1;

FIG_DIR = 'C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\Figuras';
if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end

% ============================================================
% MOTOR DC (escalon 0 -> 20 RPM, duracion 30 s)
% Trayectoria sp: [t_inicio sp_valor; ...]  (ZOH entre filas)
% ============================================================
fprintf('========== MOTOR DC ==========\n');
sp_motor = [0 20];   % escalon a 20 RPM
sim_y_plot(G_motor, sp_motor, 1.8,    0.9,    0.3,    Ts, 30, ...
    'Motor DC - PID Normal', 'RPM', 'b', ...
    fullfile(FIG_DIR, 'sim_motor_normal.png'), 0);
sim_y_plot(G_motor, sp_motor, 5.0571, 37.4598, 0.1490, Ts, 30, ...
    'Motor DC - PID Tuner',  'RPM', 'b', ...
    fullfile(FIG_DIR, 'sim_motor_tuner.png'), 0);
sim_y_plot(G_motor, sp_motor, 2.3480, 9.9974, 0.2064, Ts, 30, ...
    'Motor DC - PID BO',     'RPM', 'b', ...
    fullfile(FIG_DIR, 'sim_motor_bo.png'), 0);

% ============================================================
% HOTEND (precalentado a 150C, escalon a 200C, perturbacion 220C en t=400s)
% Duracion 1000s para ver tracking + rechazo de perturbacion
% ============================================================
fprintf('========== HOTEND ==========\n');
sp_hotend = [0 200];   % escalon a 200 (sin perturbacion)
y0_hotend = 150;                % temperatura inicial (precalentado)
sim_y_plot(G_hotend, sp_hotend, 25,      2.5,    1.5,    Ts, 1000, ...
    'Hotend - PID Normal', 'Temperatura (C)', 'r', ...
    fullfile(FIG_DIR, 'sim_hotend_normal.png'), y0_hotend);
sim_y_plot(G_hotend, sp_hotend, 55.0930, 2.9244, 0.0000, Ts, 1000, ...
    'Hotend - PID Tuner',  'Temperatura (C)', 'r', ...
    fullfile(FIG_DIR, 'sim_hotend_tuner.png'), y0_hotend);
sim_y_plot(G_hotend, sp_hotend, 35.0000, 4.0000, 3.0000, Ts, 1000, ...
    'Hotend - PID BO',     'Temperatura (C)', 'r', ...
    fullfile(FIG_DIR, 'sim_hotend_bo.png'), y0_hotend);


% ============================================================
% Funciones locales
% ============================================================
function sim_y_plot(G, sp_traj, Kp, Ki, Kd, Ts, dur, titulo, ylab, color, savepath, y0)
    es_motor = contains(titulo, 'Motor', 'IgnoreCase', true);
    DEADBAND   = 30;     % zona muerta intrínseca del motor (estudio lineal a 20 RPM)
    ALPHA_ENC  = 0.90;   % filtro paso bajo del encoder (motor)
    N_SUB      = 10;     % sub-pasos de la planta por cada Ts del control
    PWM_MIN    = 0;
    PWM_MAX    = 255;
    Y_MIN      = 0;
    if es_motor
        Y_MAX = 70;      % motor con mecanismo llega hasta ~63 RPM
    else
        Y_MAX = 280;     % hotend max antes de shutdown
    end

    % Discretizar la planta con paso fino Ts/N_SUB
    Ts_sub = Ts / N_SUB;
    Gz = c2d(G, Ts_sub, 'zoh');
    [A, B, C_out, D] = ssdata(ss(Gz));

    N = ceil(dur / Ts);
    t = (0:N-1)' * Ts;

    % Construir trayectoria de setpoint con ZOH desde sp_traj
    sp = zeros(N, 1);
    for k = 1:N
        rows = sp_traj(sp_traj(:,1) <= t(k), :);
        if isempty(rows)
            sp(k) = sp_traj(1, 2);
        else
            sp(k) = rows(end, 2);
        end
    end

    y      = zeros(N, 1);
    y_meas = zeros(N, 1);
    u      = zeros(N, 1);

    % Inicializacion: si y0 > 0 arrancamos en equilibrio del modelo (precalentado)
    if y0 > 0 && Ki > 1e-6
        Gdc  = C_out * ((eye(size(A)) - A) \ B) + D;     % ganancia DC discreta
        u_ss = y0 / Gdc;                                 % PWM que mantiene y=y0
        x    = (eye(size(A)) - A) \ (B * u_ss);          % estado de equilibrio
        y(1)      = C_out * x + D * u_ss;
        y_meas(1) = y(1);
        u(1)      = u_ss;
        integral  = u_ss / Ki;                           % integrador en equilibrio
    else
        x = zeros(size(A, 1), 1);
        integral = 0;
    end
    last_ym = y_meas(1);

    for k = 2:N
        % --- Control digital a Ts ---
        e_k     = sp(k) - y_meas(k-1);
        D_term  = -Kd * (y_meas(k-1) - last_ym) / Ts;
        integ_test = integral + e_k * Ts;
        u_unsat = Kp * e_k + Ki * integ_test + D_term;
        u(k)    = max(PWM_MIN, min(PWM_MAX, u_unsat));

        % --- Anti-windup: integracion condicional ---
        % Congela el integrador SOLO si el control esta saturado Y el error
        % empujaria mas profundo a la saturacion. Cuando e_k cambia de signo,
        % la integracion se reanuda inmediatamente (recuperacion rapida).
        deepens_sat = ((u_unsat > PWM_MAX) && (e_k > 0)) || ...
                      ((u_unsat < PWM_MIN) && (e_k < 0));
        if ~deepens_sat
            integral = integ_test;
        end
        last_ym = y_meas(k-1);

        % --- Zona muerta del motor ---
        if es_motor && u(k) < DEADBAND
            u_eff = 0;
        else
            u_eff = u(k);
        end

        % --- Planta evoluciona N_SUB sub-pasos con u_eff constante (ZOH) ---
        for j = 1:N_SUB
            x = A*x + B*u_eff;
        end
        y_raw = C_out*x + D*u_eff;
        y(k)  = max(Y_MIN, min(Y_MAX, y_raw));

        % --- Filtro del encoder/sensor ---
        if es_motor
            y_meas(k) = ALPHA_ENC * y_meas(k-1) + (1 - ALPHA_ENC) * y(k);
        else
            y_meas(k) = y(k);
        end
    end

    % --- Metricas ---
    % Fase 1: escalon inicial (hasta la perturbacion, o fin si no hay)
    if size(sp_traj, 1) >= 2
        t_pert = sp_traj(2, 1);
    else
        t_pert = dur;
    end
    idx1  = t <= t_pert;
    info1 = stepinfo(y(idx1), t(idx1), sp_traj(1, 2));
    ITAE  = trapz(t, t .* abs(sp - y));
    ess   = abs(sp(end) - y(end));
    sat_pct = 100 * mean(u >= PWM_MAX - 1e-6 | u <= PWM_MIN + 1e-6);
    fprintf('%-25s ITAE=%.4e  Ts=%.2fs  OS=%.2f%%  ess=%.4f  PWMsat=%.1f%%\n', ...
        titulo, ITAE, info1.SettlingTime, info1.Overshoot, ess, sat_pct);

    % Fase 2: respuesta a la perturbacion de setpoint (si la hay)
    if size(sp_traj, 1) >= 2
        idx2  = t >= t_pert;
        info2 = stepinfo(y(idx2), t(idx2) - t_pert, sp_traj(2, 2));
        fprintf('%25s    perturbacion -> Ts=%.2fs  OS=%.2f%%\n', ...
            '', info2.SettlingTime, info2.Overshoot);
    end

    % --- Figura: salida + senal de control (PWM) ---
    % El panel de PWM hace explicito por que el hotend no discrimina los
    % esquemas: el heater satura toda la rampa termica.
    fig = figure('Visible', 'on', 'Position', [100 100 900 420]);

    plot(t, y, [color '-'], 'LineWidth', 2); hold on;
    plot(t, sp, 'k--', 'LineWidth', 1.2);
    legend({'Salida', 'Referencia'}, 'Location', 'best');
    ylabel(ylab); xlabel('Tiempo (s)'); grid on;
    title(sprintf('Simulación: %s   (K_p=%.3f, K_i=%.3f, K_d=%.3f)', titulo, Kp, Ki, Kd));

    try
        exportgraphics(fig, savepath, 'Resolution', 150);
    catch
        saveas(fig, savepath);
    end
end
