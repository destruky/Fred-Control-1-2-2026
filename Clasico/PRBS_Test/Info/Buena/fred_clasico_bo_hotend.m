%% fred_clasico_bo_hotend.m
% Bayesian Optimization for hotend PID tuning in FrED-TEC
% -------------------------------------------------------
% This script:
% 1) Loads G_hotend from fred_modelos_pid.mat located in the same folder
% 2) Tunes Kp, Ki, Kd using Bayesian Optimization
% 3) Minimizes ITAE for a 190 °C step over 800 s with Ts = 0.1 s
%    - Simulation uses discrete plant + saturated PWM [30,255] + anti-windup
%    - Overshoot is penalized (hard cutoff at 250 °C, soft penalty per °C)
% 4) Compares the optimized PID against current FrED gains
% 5) Saves results to fred_bo_hotend_results.mat
%
% Requirements:
% - MATLAB R2022b or newer
% - Control System Toolbox
% - Statistics and Machine Learning Toolbox

clear; clc; close all;

%% 1) Build path and load model
script_dir = fileparts(mfilename('fullpath'));
mat_file   = fullfile(script_dir, 'fred_modelos_pid.mat');

if ~isfile(mat_file)
    error('Could not find fred_modelos_pid.mat in the same folder as this script.');
end

S = load(mat_file);

if ~isfield(S, 'G_hotend')
    error('The file fred_modelos_pid.mat does not contain the variable G_hotend.');
end

G_hotend = S.G_hotend;

if ~isa(G_hotend, 'tf')
    error('G_hotend must be a tf object.');
end

%% 2) Simulation settings
Ts        = 0.1;             % Sampling time of real system
T_end     = 800;             % Long horizon because hotend is slow (tau~100s)
t         = 0:Ts:T_end;      % Time vector
ref_value = 190;             % Reference temperature in °C
r         = ref_value * ones(size(t));

%% 3) Current FrED PID values (source of truth: MAIN_F.ino)
Kp_actual = 1.8;
Ki_actual = 0.9;
Kd_actual = 0.3;

%% 4) Bayesian Optimization search variables
vars = [
    optimizableVariable('Kp', [0.1, 10], 'Type', 'real')
    optimizableVariable('Ki', [0.01,  5], 'Type', 'real')
    optimizableVariable('Kd', [0.0,   2], 'Type', 'real')
];

%% 5) Evaluate current PID for comparison
ITAE_actual = computeITAE(Kp_actual, Ki_actual, Kd_actual, G_hotend, t, r);

fprintf('Current FrED PID:\n');
fprintf('  Kp = %.6f\n', Kp_actual);
fprintf('  Ki = %.6f\n', Ki_actual);
fprintf('  Kd = %.6f\n', Kd_actual);
fprintf('  ITAE_actual = %.6f\n\n', ITAE_actual);

%% 6) Objective function for Bayesian Optimization
objFcn = @(x) objectiveFcn(x, G_hotend, t, r);

%% 7) Run Bayesian Optimization
% Seed with current known-good PID so BO starts from a valid operating point
% and guarantees the result is at least as good as the current gains.
results = bayesopt(objFcn, vars, ...
    'MaxObjectiveEvaluations', 60, ...
    'InitialX', table(Kp_actual, Ki_actual, Kd_actual), ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'IsObjectiveDeterministic', true, ...
    'Verbose', 1);

%% 8) Extract best PID gains
bestX = bestPoint(results);

Kp_bo = bestX.Kp;
Ki_bo = bestX.Ki;
Kd_bo = bestX.Kd;

ITAE_bo = computeITAE(Kp_bo, Ki_bo, Kd_bo, G_hotend, t, r);

fprintf('Optimized PID from Bayesian Optimization:\n');
fprintf('  Kp_bo = %.6f\n', Kp_bo);
fprintf('  Ki_bo = %.6f\n', Ki_bo);
fprintf('  Kd_bo = %.6f\n', Kd_bo);
fprintf('  ITAE_bo = %.6f\n\n', ITAE_bo);

%% 9) Build response curves for plotting
% Uses the same discrete saturated simulation for a fair comparison.
Tf = 5 * Ts;   % derivative filter time constant (N=5), same used in computeITAE

y_actual = simLoop(Kp_actual, Ki_actual, Kd_actual, Tf, G_hotend, t, ref_value);
y_bo     = simLoop(Kp_bo,     Ki_bo,     Kd_bo,     Tf, G_hotend, t, ref_value);

%% 10) Plot optimized vs current PID
figure('Color', 'w');
plot(t, r,        'k--', 'LineWidth', 1.5); hold on;
plot(t, y_actual, 'b',   'LineWidth', 1.8);
plot(t, y_bo,     'r',   'LineWidth', 1.8);
yline(250, 'm--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Hotend Response: Current PID vs Bayesian-Optimized PID');
legend('Reference = 190°C', 'Current FrED PID', 'Optimized PID', ...
       'Safety limit 250°C', 'Location', 'best');

%% 11) Save results
save_file = fullfile(script_dir, 'fred_bo_hotend_results.mat');
save(save_file, 'Kp_bo', 'Ki_bo', 'Kd_bo', 'ITAE_bo', 'ITAE_actual');

fprintf('Results saved to:\n  %s\n', save_file);

%% =============================================================
% Local functions
% =============================================================

function J = objectiveFcn(x, G_hotend, t, r)
    % Wrapper for bayesopt — returns penalty on any error or invalid result.
    try
        J = computeITAE(x.Kp, x.Ki, x.Kd, G_hotend, t, r);
    catch
        J = 1e6;
    end
    if ~isfinite(J) || isnan(J)
        J = 1e6;
    end
end

% ---------------------------------------------------------------
function ITAE = computeITAE(Kp, Ki, Kd, G_hotend, t, r)
% Simulates the closed loop with a discrete plant, saturated PWM output
% [30,255], anti-windup, and a first-order derivative filter (N=5).
% Returns ITAE + overshoot penalty (hard cutoff at 250°C).

    ref_val = r(1);
    y = simLoop(Kp, Ki, Kd, 5*(t(2)-t(1)), G_hotend, t, ref_val);

    % Hard safety cutoff: never allow temperatures near the shutdown limit
    if max(y) > 250
        ITAE = 1e6;
        return;
    end

    % Soft overshoot penalty: 500 s²·°C per °C above setpoint
    overshoot_C = max(0, max(y) - ref_val);

    e    = r - y;
    ITAE = trapz(t, t .* abs(e)) + 500 * overshoot_C;

    if ~isfinite(ITAE) || isnan(ITAE) || any(~isfinite(y))
        ITAE = 1e6;
    end
end

% ---------------------------------------------------------------
function y = simLoop(Kp, Ki, Kd, Tf, G_hotend, t, ref_val)
% Discrete closed-loop simulation with:
%   - ZOH-discretized plant at Ts = t(2)-t(1)
%   - First-order derivative filter: C_d(z) ≈ pid(Kp,Ki,Kd,Tf)
%   - PWM saturation: [30, 255]
%   - Conditional anti-windup: integrator freezes when output is saturated

    Ts  = t(2) - t(1);
    n   = length(t);

    % Discretize plant
    G_d = c2d(G_hotend, Ts, 'zoh');
    [Ad, Bd, Cd, Dd] = ssdata(ss(G_d));
    nx = size(Ad, 1);

    y     = zeros(1, n);
    x     = zeros(nx, 1);
    e_int = 0;       % integral state
    e_prv = 0;       % previous error for derivative
    d_f   = 0;       % derivative filter state

    PWM_MIN = 30;
    PWM_MAX = 255;

    for k = 1:n
        e = ref_val - y(k);

        % Derivative: first-order filter  d_f(k) = [Tf*d_f(k-1) + Kd*de/dt] / (Tf+Ts)
        d_raw = (e - e_prv) / Ts;
        d_f   = (Tf * d_f + Kd * d_raw) / (Tf + Ts);

        % PID control signal
        u_raw = Kp * e + Ki * e_int + d_f;

        % Saturate to physical PWM range
        u_sat = min(PWM_MAX, max(PWM_MIN, u_raw));

        % Anti-windup: only integrate when output is not saturated
        if u_raw == u_sat
            e_int = e_int + e * Ts;
        end

        e_prv = e;

        % Advance plant state
        if k < n
            x      = Ad * x + Bd * u_sat;
            y(k+1) = Cd * x + Dd * u_sat;
        end
    end
end
