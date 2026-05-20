% generar_figuras_reporte.m
% Genera las figuras de SIMULACION / VALIDACION que faltan para el
% Reporte Clasico (todas menos las de hardware fisico y BO adaptativo):
%
%   Figura B  -> figB_prbs_motor.png        senal PRBS + RPM (datos crudos)
%   Figura C  -> figC_escalones_hotend.png  escalones PWM + temp (datos crudos)
%   Figura D  -> figD_validacion_Gmotor.png G(s) motor vs datos de TEST
%   Figura E  -> figE_validacion_Ghotend.png G(s) hotend vs datos reales
%   Figura 1  -> fig1_sim_motor_overlay.png  3 esquemas PID motor (overlay)
%   Figura 2  -> fig2_sim_hotend_overlay.png 3 esquemas PID hotend (overlay)
%
% Ejecutar desde Clasico/MATLAB/ con la carpeta Info/Buena en el path.

clear; clc; close all;

DATA_DIR = fullfile('..', 'PRBS_Test', 'Info', 'Buena');
FIG_DIR  = fullfile('..', 'Resultados', 'Figuras');
if ~exist(FIG_DIR, 'dir'), mkdir(FIG_DIR); end

load(fullfile(DATA_DIR, 'fred_modelos_pid.mat'));   % G_motor, G_hotend
Ts = 0.1;

% ============================================================
% FIGURA B — Senal PRBS aplicada al motor y respuesta de RPM
% ============================================================
m = readtable('C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\PRBSmotorfinal.csv');
tB = m.t_ms / 1000;
fig = figure('Visible','off','Position',[100 100 900 520]);
subplot(2,1,1);
stairs(tB, m.pwm, 'b-', 'LineWidth', 1);
ylabel('PWM'); title('Figura B — Senal PRBS aplicada al motor DC (dataset de identificacion)');
grid on; xlim([tB(1) tB(end)]);
subplot(2,1,2);
plot(tB, m.rpm, 'r-', 'LineWidth', 1);
xlabel('Tiempo (s)'); ylabel('RPM'); title('Respuesta de velocidad medida');
grid on; xlim([tB(1) tB(end)]);
exportgraphics(fig, fullfile(FIG_DIR,'figB_prbs_motor.png'), 'Resolution',150);
close(fig);
fprintf('Figura B generada.\n');

% ============================================================
% FIGURA C — Escalones de PWM al hotend y respuesta de temperatura
% ============================================================
files  = {'3)hotend60.csv','1)hotend100.csv','2)hotend180.csv','4)hotend220.csv'};
pwmlab = {'PWM 60','PWM 100','PWM 180','PWM 220'};
cols   = lines(4);
fig = figure('Visible','off','Position',[100 100 900 520]);
subplot(2,1,1); hold on;
subplot(2,1,2); hold on;
for i = 1:4
    h = readtable(fullfile(DATA_DIR, files{i}));
    v = ~isnan(h.temp_C) & ~isnan(h.pwm_heater);
    th = h.t_ms(v)/1000;
    subplot(2,1,1);
    stairs(th, h.pwm_heater(v), 'Color', cols(i,:), 'LineWidth', 1, ...
        'DisplayName', pwmlab{i});
    subplot(2,1,2);
    plot(th, h.temp_C(v), 'Color', cols(i,:), 'LineWidth', 1.2, ...
        'DisplayName', pwmlab{i});
end
subplot(2,1,1);
ylabel('PWM'); title('Figura C — Escalones de PWM aplicados al hotend');
legend('Location','best'); grid on;
subplot(2,1,2);
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title('Respuesta de temperatura'); legend('Location','best'); grid on;
exportgraphics(fig, fullfile(FIG_DIR,'figC_escalones_hotend.png'), 'Resolution',150);
close(fig);
fprintf('Figura C generada.\n');

% ============================================================
% FIGURA D — Validacion G(s) motor vs holdout 30% de PRBSmotorfinal
%   Mismo dataset, ventana y split (70/30) que fred_clasico_motor_tfest.m,
%   por eso el FIT reproduce el ~64.7% reportado en la Tabla 1.
% ============================================================
mD  = readtable('C:\Users\rquin\OneDrive\Desktop\Acceso a Claudio\FrED-TEC\Clasico\Resultados\CSVs\PRBSmotorfinal.csv');
mwD = mD(300:end, :);                  % misma ventana limpia (salta 30s de arranque)
uD  = detrend(mwD.pwm, 0);             % detrend igual que en la identificacion
yD  = detrend(mwD.rpm, 0);
ymeanD = mean(mwD.rpm);                % media para graficar en RPM reales
ND  = numel(yD);  NtrD = round(0.70*ND);
uVal = uD(NtrD+1:end);  yVal = yD(NtrD+1:end);
tVal = (0:numel(yVal)-1)' * Ts;
yVal_sim = lsim(G_motor, uVal, tVal);
rmse_m = sqrt(mean((yVal_sim - yVal).^2));
fit_m  = 100*(1 - norm(yVal - yVal_sim)/norm(yVal - mean(yVal)));
fprintf('Validacion Motor (holdout 30%% PRBSmotorfinal): RMSE=%.2f RPM  FIT=%.1f%%\n', rmse_m, fit_m);
fig = figure('Visible','off','Position',[100 100 900 420]);
plot(tVal, yVal + ymeanD, 'r-', 'LineWidth', 1, 'DisplayName','Real (holdout 30%)'); hold on;
plot(tVal, yVal_sim + ymeanD, 'b--', 'LineWidth', 1.5, 'DisplayName','G(s) simulado');
xlabel('Tiempo (s)'); ylabel('RPM');
title(sprintf('Figura D — Validacion G(s) Motor DC   (RMSE=%.2f RPM, FIT=%.1f%%)', rmse_m, fit_m));
legend('Location','best'); grid on;
exportgraphics(fig, fullfile(FIG_DIR,'figD_validacion_Gmotor.png'), 'Resolution',150);
close(fig);
fprintf('Figura D generada.\n');

% ============================================================
% FIGURA E — Validacion G(s) hotend vs los 2 escalones de holdout
%   PWM 60 y PWM 220: ninguno se uso en el ajuste (train = merge(h100,h180)).
%   Mostrar ambos prueba el modelo en todo el rango (baja y alta temp).
% ============================================================
valE = {'3)hotend60.csv', '4)hotend220.csv'};
labE = {'PWM 60', 'PWM 220'};
colE = {[0 0.45 0.74], [0.85 0.33 0.10]};   % azul, naranja
r2E = zeros(1,2);  rmseE = zeros(1,2);
fig = figure('Visible','off','Position',[100 100 900 460]); hold on;
for i = 1:2
    hE  = readtable(fullfile(DATA_DIR, valE{i}));
    v   = ~isnan(hE.temp_C) & ~isnan(hE.pwm_heater);
    thE = hE.t_ms(v)/1000;  uhE = double(hE.pwm_heater(v));  yhE = double(hE.temp_C(v));
    yhE_sim  = lsim(G_hotend, uhE, thE) + yhE(1);
    rmseE(i) = sqrt(mean((yhE_sim - yhE).^2));
    r2E(i)   = 1 - sum((yhE - yhE_sim).^2) / sum((yhE - mean(yhE)).^2);
    plot(thE, yhE,     '-',  'Color', colE{i}, 'LineWidth', 1.4, ...
        'DisplayName', sprintf('%s real', labE{i}));
    plot(thE, yhE_sim, '--', 'Color', colE{i}, 'LineWidth', 1.6, ...
        'DisplayName', sprintf('%s modelo G(s)', labE{i}));
end
fprintf('Validacion Hotend holdout: PWM60 R2=%.4f RMSE=%.2f C | PWM220 R2=%.4f RMSE=%.2f C\n', ...
    r2E(1), rmseE(1), r2E(2), rmseE(2));
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title(sprintf('Figura E — Validacion G(s) Hotend   (R^2: PWM 60=%.3f, PWM 220=%.3f)', r2E(1), r2E(2)));
legend('Location','best'); grid on;
exportgraphics(fig, fullfile(FIG_DIR,'figE_validacion_Ghotend.png'), 'Resolution',150);
close(fig);
fprintf('Figura E generada.\n');

% ============================================================
% FIGURA 1 — Overlay 3 esquemas PID motor
% ============================================================
[t_n,y_n] = simular(G_motor,[0 20],   1.8,    0.9,    0.3,    Ts,60,0);
[~,  y_t] = simular(G_motor,[0 20],   5.0571, 37.4598,0.1490, Ts,60,0);
[~,  y_b] = simular(G_motor,[0 20],   2.3480, 9.9974, 0.2064, Ts,60,0);
fig = figure('Visible','off','Position',[100 100 900 460]);
plot(t_n,y_n,'b-','LineWidth',1.8,'DisplayName','PID Normal'); hold on;
plot(t_n,y_t,'r-','LineWidth',1.8,'DisplayName','PID Tuner');
plot(t_n,y_b,'g-','LineWidth',1.8,'DisplayName','BO Fijo');
yline(20,'k--','Referencia 20 RPM','HandleVisibility','off');
xlabel('Tiempo (s)'); ylabel('RPM');
title('Figura 1 — Respuesta escalon Simulink: Motor DC (3 esquemas PID)');
legend('Location','best'); grid on;
exportgraphics(fig, fullfile(FIG_DIR,'fig1_sim_motor_overlay.png'), 'Resolution',150);
close(fig);
fprintf('Figura 1 generada.\n');

% ============================================================
% FIGURA 2 — Overlay 3 esquemas PID hotend (escalon a 200 C)
% ============================================================
spH = [0 200];
[t_n,y_n] = simular(G_hotend,spH, 25,      2.5,    1.5,   Ts,1000,150);
[~,  y_t] = simular(G_hotend,spH, 55.0930, 2.9244, 0.0000,Ts,1000,150);
[~,  y_b] = simular(G_hotend,spH, 35.0000, 4.0000, 3.0000,Ts,1000,150);
fig = figure('Visible','off','Position',[100 100 900 460]);
plot(t_n,y_n,'b-','LineWidth',1.6,'DisplayName','PID Normal'); hold on;
plot(t_n,y_t,'r-','LineWidth',1.6,'DisplayName','PID Tuner');
plot(t_n,y_b,'g-','LineWidth',1.6,'DisplayName','BO Fijo');
yline(200,'k--','Referencia 200 C','HandleVisibility','off');
xlabel('Tiempo (s)'); ylabel('Temperatura (C)');
title('Figura 2 — Respuesta escalon Simulink: Hotend (3 esquemas PID)');
legend('Location','best'); grid on;
exportgraphics(fig, fullfile(FIG_DIR,'fig2_sim_hotend_overlay.png'), 'Resolution',150);
close(fig);
fprintf('Figura 2 generada.\n');

fprintf('\nTodas las figuras en %s\n', FIG_DIR);


% ============================================================
% Funcion local — nucleo de simulacion (identico a simular_realista.m)
% ============================================================
function [t, y] = simular(G, sp_traj, Kp, Ki, Kd, Ts, dur, y0)
    es_motor  = (max(sp_traj(:,2)) <= 60);   % motor: setpoint en RPM (<=60); hotend: en C
    DEADBAND  = 30;  ALPHA_ENC = 0.90;  N_SUB = 10;
    PWM_MIN = 0;  PWM_MAX = 255;  Y_MIN = 0;
    if es_motor, Y_MAX = 55; else, Y_MAX = 280; end

    Gz = c2d(G, Ts/N_SUB, 'zoh');
    [A,B,C_out,D] = ssdata(ss(Gz));
    N = ceil(dur/Ts);  t = (0:N-1)'*Ts;

    sp = zeros(N,1);
    for k = 1:N
        r = sp_traj(sp_traj(:,1) <= t(k), :);
        if isempty(r), sp(k) = sp_traj(1,2); else, sp(k) = r(end,2); end
    end

    y = zeros(N,1);  y_meas = zeros(N,1);  u = zeros(N,1);
    if y0 > 0 && Ki > 1e-6
        Gdc  = C_out*((eye(size(A))-A)\B) + D;
        u_ss = y0/Gdc;
        x    = (eye(size(A))-A)\(B*u_ss);
        y(1) = C_out*x + D*u_ss;  y_meas(1) = y(1);  u(1) = u_ss;
        integral = u_ss/Ki;
    else
        x = zeros(size(A,1),1);  integral = 0;
    end
    last_ym = y_meas(1);

    for k = 2:N
        e_k     = sp(k) - y_meas(k-1);
        D_term  = -Kd*(y_meas(k-1) - last_ym)/Ts;
        integ_test = integral + e_k*Ts;
        u_unsat = Kp*e_k + Ki*integ_test + D_term;
        u(k)    = max(PWM_MIN, min(PWM_MAX, u_unsat));
        deepens = ((u_unsat>PWM_MAX)&&(e_k>0)) || ((u_unsat<PWM_MIN)&&(e_k<0));
        if ~deepens, integral = integ_test; end
        last_ym = y_meas(k-1);

        if es_motor && u(k) < DEADBAND, u_eff = 0; else, u_eff = u(k); end
        for j = 1:N_SUB, x = A*x + B*u_eff; end
        y(k) = max(Y_MIN, min(Y_MAX, C_out*x + D*u_eff));
        if es_motor
            y_meas(k) = ALPHA_ENC*y_meas(k-1) + (1-ALPHA_ENC)*y(k);
        else
            y_meas(k) = y(k);
        end
    end
end
