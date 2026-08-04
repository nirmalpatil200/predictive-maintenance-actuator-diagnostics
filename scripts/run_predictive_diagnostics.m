%% run_predictive_diagnostics.m
% Runs the actuator fault-injection model, then performs signal processing
% and anomaly detection on the logged motor current:
%   1. Resample to a fixed sample rate (variable-step solver output isn't
%      uniformly spaced, and FFT/rolling-RMS both need a fixed Fs)
%   2. Rolling RMS over a 0.5s window
%   3. 3-sigma threshold from a healthy baseline window (t = 0 to 6s)
%   4. First-alarm detection + latency relative to each injected fault
%   5. FFT comparison: healthy window vs. each faulted window
%   6. Diagnostics dashboard figure, saved to ../results

clear; clc; close all;

modelName = 'Actuator_FaultInjection_Model';

%% 0. Load model parameters (needed because Simulink block parameters
%    reference these by name, and `clear` above wipes the base workspace)
modelsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
load(fullfile(modelsDir, 'actuator_params.mat'));

%% 1. Run simulation
out = sim(modelName, 'StopTime', '20');
t_raw    = out.motor_current.Time;
curr_raw = out.motor_current.Data;

%% 2. Resample to a fixed sample rate
Fs = 10000; % [Hz]
time = (t_raw(1):1/Fs:t_raw(end))';
current = interp1(t_raw, curr_raw, time, 'linear');

%% 3. Rolling RMS (0.5s window)
win_size = round(0.5 * Fs);
curr_rms = sqrt(movmean(current.^2, [win_size-1, 0]));
%% 4. Baseline (healthy: t = 0 to 6s) and 3-sigma threshold
idx_healthy = time >= 2.0 & time <= 6.0;
mu_base    = mean(curr_rms(idx_healthy));
sigma_base = std(curr_rms(idx_healthy));
thresh_alarm = mu_base + 3.5 * sigma_base;

fprintf('Baseline healthy RMS: mean=%.4f A, std=%.4f A\n', mu_base, sigma_base);
fprintf('3-sigma alarm threshold: %.4f A\n', thresh_alarm);

%% 5. First anomaly trigger + latency relative to each fault
fault_times = [8.0, 14.0];
fault_names = {'Friction (bearing wear)', 'Thermal (stator resistance)'};

alarm_idx = find(curr_rms > thresh_alarm & time > 6.0, 1, 'first');
if isempty(alarm_idx)
    warning('No anomaly crossed the 3-sigma threshold. Check gains/threshold.');
    alarm_time = NaN;
else
    alarm_time = time(alarm_idx);
    fprintf('ANOMALY DETECTED at t = %.3f s (RMS = %.3f A, threshold = %.3f A)\n', ...
        alarm_time, curr_rms(alarm_idx), thresh_alarm);

    % Attribute to the most recent fault at or before the alarm
    prior_faults = fault_times(fault_times <= alarm_time);
    if ~isempty(prior_faults)
        nearest_fault = max(prior_faults);
        fault_label = fault_names{fault_times == nearest_fault};
        dt_detect = alarm_time - nearest_fault;
        fprintf('Attributed to: %s (injected at t=%.1fs)\n', fault_label, nearest_fault);
        fprintf('Detection latency (dt_detect): %.3f s\n', dt_detect);
    end
end

%% 6. FFT: healthy vs. each faulted window
L = 2048;
mask_healthy  = time >= 2  & time <= 4;
mask_friction = time >= 9  & time <= 11;
mask_thermal  = time >= 16 & time <= 18;

fft_healthy  = abs(fft(current(mask_healthy)  - mean(current(mask_healthy)),  L));
fft_friction = abs(fft(current(mask_friction) - mean(current(mask_friction)), L));
fft_thermal  = abs(fft(current(mask_thermal)  - mean(current(mask_thermal)),  L));
f_axis = (0:(L/2)) * (Fs / L);

%% 7. Diagnostics dashboard figure
fig = figure('Name', 'Predictive Diagnostics Dashboard', 'Position', [100, 100, 950, 650]);

subplot(2,1,1);
plot(time, curr_rms, 'b-', 'LineWidth', 1.3); hold on;
yline(thresh_alarm, 'r--', '3-Sigma Threshold', 'LineWidth', 1.5);
xline(8.0,  'k:', 'Friction Fault (t=8s)');
xline(14.0, 'k:', 'Thermal Fault (t=14s)');
if ~isempty(alarm_idx)
    plot(alarm_time, curr_rms(alarm_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
title('Actuator Motor Current — Rolling RMS & Anomaly Alarm');
xlabel('Time (s)'); ylabel('RMS Current (A)'); grid on;

subplot(2,1,2);
plot(f_axis, fft_healthy(1:L/2+1),  'g-', 'LineWidth', 1.2); hold on;
plot(f_axis, fft_friction(1:L/2+1), 'm-', 'LineWidth', 1.2);
plot(f_axis, fft_thermal(1:L/2+1),  'r-', 'LineWidth', 1.2);
title('FFT Spectrum: Healthy vs. Friction Fault vs. Thermal Fault');
xlabel('Frequency (Hz)'); ylabel('Magnitude'); xlim([0 500]);
legend('Healthy (2-4s)', 'Friction fault (9-11s)', 'Thermal fault (16-18s)', 'Location', 'best');
grid on;

resultsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end
saveas(fig, fullfile(resultsDir, 'Predictive_Maintenance_Dashboard.png'));

disp('Diagnostics pipeline completed successfully.');
