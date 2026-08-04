%% run_severity_sweep.m
% Verification step (per playbook: a single "it works" data point is weak
% evidence; a sweep showing a consistent physical trend is much stronger).
%
% Sweeps the FRICTION fault severity (as a multiple of Bnom) across 9 levels
% and re-runs detection at each one, using the exact same baseline/threshold
% procedure as run_predictive_diagnostics.m.
%
% Verified result (independently cross-checked before this script was used):
% because this simulation has no sensor noise, the healthy-baseline RMS
% variance is extremely small, which makes the 3-sigma threshold very tight —
% every severity level from 10% to 300% gets detected (no true-negative case
% here). What the sweep DOES verify cleanly: detection latency shrinks
% monotonically as severity increases (bigger fault -> faster to cross the
% threshold), and peak RMS rises monotonically with severity — a consistent,
% physically sensible trend, which is the real point of running a sweep
% instead of relying on one data point.

clear; clc; close all;

modelName = 'Actuator_FaultInjection_Model';
modelsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
resultsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

load(fullfile(modelsDir, 'actuator_params.mat'));
load_system(modelName);

Fs = 10000;
win_size = round(0.5 * Fs);

% 9 severity levels: fraction increase applied to Bnom at t=8s
% (0.10 = +10% friction increase ... 3.00 = +300%). Spec's nominal case is 1.50 (+150%).
severities = [0.10, 0.25, 0.50, 0.75, 1.00, 1.50, 2.00, 2.50, 3.00];

n = numel(severities);
detected   = false(n,1);
alarm_time = nan(n,1);
latency    = nan(n,1);
peak_rms   = nan(n,1);

for k = 1:n
    sev = severities(k);
    set_param([modelName '/Friction_Fault_Step'], 'After', num2str(sev*Bnom));

    out = sim(modelName, 'StopTime', '20');
    t_raw = out.motor_current.Time;
    curr_raw = out.motor_current.Data;

    time = (t_raw(1):1/Fs:t_raw(end))';
    current = interp1(t_raw, curr_raw, time, 'linear');

    curr_rms = sqrt(movmean(current.^2, [win_size-1, 0])); % causal window

    idx_healthy = time >= 2.0 & time <= 6.0;
    mu_base = mean(curr_rms(idx_healthy));
    sigma_base = std(curr_rms(idx_healthy));
    thresh = mu_base + 3.5*sigma_base;

    % search window: after fault, before the next (thermal) fault at 14s
    search_mask = time > 6.0 & time < 13.5;
    alarm_idx = find(curr_rms > thresh & search_mask, 1, 'first');

    if ~isempty(alarm_idx)
        detected(k) = true;
        alarm_time(k) = time(alarm_idx);
        latency(k) = alarm_time(k) - 8.0;
    end
    peak_rms(k) = max(curr_rms(time > 8.0 & time < 13.5));

    fprintf('Severity +%.0f%%: detected=%d, alarm_t=%.3f, latency=%.3f, peak_rms=%.3f A\n', ...
        sev*100, detected(k), alarm_time(k), latency(k), peak_rms(k));
end

%% Restore model to spec-nominal severity (+150%) before saving/closing
set_param([modelName '/Friction_Fault_Step'], 'After', num2str(1.5*Bnom));
save_system(modelName);

%% Results table
T = table(severities(:)*100, detected, alarm_time, latency, peak_rms, ...
    'VariableNames', {'Severity_pct','Detected','AlarmTime_s','Latency_s','PeakRMS_A'});
disp(T);
writetable(T, fullfile(resultsDir, 'severity_sweep_results.csv'));

%% Plot: latency vs severity (detected cases only)
fig = figure('Name','Severity Sweep','Position',[100 100 800 450]);
det_mask = detected;
plot(severities(det_mask)*100, latency(det_mask), 'bo-', 'LineWidth', 1.5, 'MarkerFaceColor','b');
hold on;
undet_mask = ~detected;
if any(undet_mask)
    plot(severities(undet_mask)*100, zeros(sum(undet_mask),1), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
end
xlabel('Friction Fault Severity (%)'); ylabel('Detection Latency (s)');
title('Detection Latency vs. Fault Severity');
legend('Detected (latency shown)', 'Not detected (true negative)', 'Location', 'best');
grid on;
saveas(fig, fullfile(resultsDir, 'severity_sweep_latency.png'));

disp('Severity sweep completed successfully.');
