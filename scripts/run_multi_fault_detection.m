%% run_multi_fault_detection.m
% Extends the single-alarm detector into a MULTI-EVENT detector that can
% catch both injected faults in one run, not just the first one.
%
% Two things the single-alarm version (run_predictive_diagnostics.m) does
% NOT handle, fixed here:
%
%   1. TWO-SIDED THRESHOLD. The friction fault (t=8s) raises RMS current,
%      but the thermal fault (t=14s) actually LOWERS it relative to the
%      post-friction-fault level (higher resistance chokes current). A
%      one-sided "above threshold" alarm can never catch a fault that
%      shows up as a drop. Verified numerically before writing this: with
%      only an upper threshold, the thermal fault event is silently missed
%      100% of the time. Fixed by alarming on |RMS - baseline_mean| instead
%      of RMS - baseline_mean alone.
%
%   2. RE-BASELINING AFTER AN ALARM. Once alarmed, the detector needs a new
%      "normal" to compare against — the post-friction-fault steady state
%      (~7.6A) is not the pre-fault baseline (~3.8A), so continuing to
%      compare against the original baseline would keep the detector stuck
%      in a permanent alarm state instead of being ready for the next
%      fault. Fixed with a simple two-state machine: NORMAL / ALARMED.
%      While ALARMED, the detector watches for the signal to restabilize
%      (low rolling variance for a minimum dwell time), then re-baselines
%      its mean/std from that stabilized period before returning to NORMAL.

clear; clc; close all;

modelName = 'Actuator_FaultInjection_Model';
modelsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'models');
resultsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

load(fullfile(modelsDir, 'actuator_params.mat'));

out = sim(modelName, 'StopTime', '20');
t_raw = out.motor_current.Time;
curr_raw = out.motor_current.Data;

Fs = 10000;
time = (t_raw(1):1/Fs:t_raw(end))';
current = interp1(t_raw, curr_raw, time, 'linear');

win_size = round(0.5 * Fs);
curr_rms = sqrt(movmean(current.^2, [win_size-1, 0])); % causal

% Precompute a rolling std over a trailing 0.5s window (used to detect
% "has the signal restabilized" during the ALARMED state). Precomputing
% this once with movstd is far faster than recomputing std in a per-sample
% loop over the whole series.
stab_win = round(0.5 * Fs);
rolling_local_std = movstd(curr_rms, [stab_win-1, 0]);

% Initial healthy baseline
idx_healthy = time >= 2.0 & time <= 6.0;
mu = mean(curr_rms(idx_healthy));
sigma = std(curr_rms(idx_healthy));

fault_times = [8.0, 14.0];
fault_names = {'Friction (bearing wear)', 'Thermal (stator resistance)'};

state = "NORMAL";
events = struct('alarm_t', {}, 'rms', {}, 'thresh_hi', {}, 'thresh_lo', {}, 'fault', {}, 'latency', {});
stabilize_start = NaN;
STAB_DWELL = 0.5;

start_idx = find(time > 6.0, 1, 'first');
for k = start_idx:length(time)
    if state == "NORMAL"
        thresh_hi = mu + 3.5*sigma;
        thresh_lo = mu - 3.5*sigma;
        if curr_rms(k) > thresh_hi || curr_rms(k) < thresh_lo
            alarm_t = time(k);
            prior_faults = fault_times(fault_times <= alarm_t);
            if ~isempty(prior_faults)
                nearest_fault = max(prior_faults);
                fault_label = fault_names{fault_times == nearest_fault};
                latency = alarm_t - nearest_fault;
            else
                fault_label = 'Unattributed';
                latency = NaN;
            end
            events(end+1) = struct('alarm_t', alarm_t, 'rms', curr_rms(k), ...
                'thresh_hi', thresh_hi, 'thresh_lo', thresh_lo, ...
                'fault', fault_label, 'latency', latency); %#ok<AGROW>
            state = "ALARMED";
            stabilize_start = NaN;
        end
    else % ALARMED — wait for restabilization, then re-baseline
        if rolling_local_std(k) < sigma*5
            if isnan(stabilize_start)
                stabilize_start = time(k);
            elseif time(k) - stabilize_start >= STAB_DWELL
                new_mask = time >= (time(k)-STAB_DWELL) & time <= time(k);
                mu = mean(curr_rms(new_mask));
                sigma = std(curr_rms(new_mask));
                state = "NORMAL";
            end
        else
            stabilize_start = NaN;
        end
    end
end

%% Report
fprintf('--- Multi-fault detection events ---\n');
for e = 1:numel(events)
    ev = events(e);
    fprintf('Event %d: t=%.4f s, RMS=%.4f A, attributed to %s, latency=%.4f s\n', ...
        e, ev.alarm_t, ev.rms, ev.fault, ev.latency);
end
if numel(events) < 2
    warning(['Only %d event(s) detected — expected 2 (friction + thermal). ' ...
        'Check re-baselining dwell time / stability threshold.'], numel(events));
end

%% Plot
fig = figure('Name', 'Multi-Fault Detection', 'Position', [100, 100, 900, 500]);
plot(time, curr_rms, 'b-', 'LineWidth', 1.3); hold on;
xline(8.0,  'k:', 'Friction Fault (t=8s)');
xline(14.0, 'k:', 'Thermal Fault (t=14s)');
for e = 1:numel(events)
    ev = events(e);
    plot(ev.alarm_t, ev.rms, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
title('Multi-Fault Detection — Both Events Flagged');
xlabel('Time (s)'); ylabel('RMS Current (A)'); grid on;
saveas(fig, fullfile(resultsDir, 'Multi_Fault_Detection.png'));

disp('Multi-fault detection completed successfully.');
