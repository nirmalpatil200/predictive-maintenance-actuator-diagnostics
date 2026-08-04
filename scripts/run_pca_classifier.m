%% run_pca_classifier.m
% Satisfies the spec's requirement to use the Statistics & Machine Learning
% Toolbox (Section 2) for anomaly classification, via PCA.
%
% HONEST NOTE, stated up front rather than discovered later: this model's
% current signal is nearly flat within any short window (no sensor noise,
% no AC ripple), which makes derived features (mean, RMS, std, peak) almost
% perfectly collinear — PC1 alone explains ~100% of baseline variance.
% That means PCA here is mathematically valid but doesn't reveal hidden
% multivariate structure the way it would on a richer, noisier real signal
% — PC1's score ends up tracking essentially the same "current level"
% information that the RMS detector already uses. This script is a
% legitimate, working PCA classifier (and does use the required toolbox),
% but it should be understood as demonstrating the *method*, not as an
% independently more powerful detector than the RMS-based one for THIS
% particular dataset.
%
% This is a SINGLE-BASELINE classifier (unlike run_multi_fault_detection.m,
% it does not re-baseline after the first event) — it answers "has this
% run departed from healthy operation," and stays flagged for the
% remainder of the run once it does. For the multi-event (both faults,
% separately re-armed) detector, see run_multi_fault_detection.m.

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

%% 1. Windowed feature extraction (0.5s trailing window, stepped every 0.1s)
win_samples = round(0.5*Fs);
step_samples = round(0.1*Fs);
starts = 1:step_samples:(length(current)-win_samples);
nWin = numel(starts);
feat_time = zeros(nWin,1);
features  = zeros(nWin,4); % [mean(|x|), RMS, std, max(|x|)]

for w = 1:nWin
    seg = current(starts(w):starts(w)+win_samples-1);
    features(w,:) = [mean(abs(seg)), sqrt(mean(seg.^2)), std(seg), max(abs(seg))];
    feat_time(w) = time(starts(w)+win_samples-1);
end

%% 2. Fit PCA on the healthy baseline window (t = 2 to 6s) using the
%    Statistics and Machine Learning Toolbox's pca() function
idx_base = feat_time >= 2.0 & feat_time <= 6.0;
mu_f = mean(features(idx_base,:));
sigma_f = std(features(idx_base,:));

% Regularize: floor the standard deviation at 1% of that feature's own
% baseline mean magnitude. Without this, near-zero-variance features
% (like 'std', which is ~0 for a near-flat signal) get divided by an
% almost-zero number during standardization, amplifying floating-point
% noise into spurious, meaningless large scores — verified this happens
% before adding the floor.
floor_val = 0.01 * abs(mu_f);
sigma_f = max(sigma_f, floor_val);

features_norm = (features - mu_f) ./ sigma_f;
baseline_norm = (features(idx_base,:) - mu_f) ./ sigma_f;

[coeff, ~, ~, ~, explained] = pca(baseline_norm);
fprintf('PCA explained variance by component: %s\n', mat2str(round(explained',2)));

pc1 = coeff(:,1);
scores = features_norm * pc1;

%% 3. Threshold on PC1 score using the healthy baseline
base_scores = scores(idx_base);
mu_s = mean(base_scores);
sigma_s = std(base_scores);
thresh_hi = mu_s + 3.5*sigma_s;
thresh_lo = mu_s - 3.5*sigma_s;

fprintf('PC1 score baseline: mean=%.4f, std=%.4f\n', mu_s, sigma_s);
fprintf('PC1 score threshold: [%.4f, %.4f]\n', thresh_lo, thresh_hi);

%% 4. First anomaly crossing
after_baseline = feat_time > 6.0;
cross_idx = find((scores > thresh_hi | scores < thresh_lo) & after_baseline, 1, 'first');
if isempty(cross_idx)
    warning('PCA classifier found no anomaly. Check features/threshold.');
else
    cross_t = feat_time(cross_idx);
    fprintf('PCA ANOMALY FLAGGED at t = %.4f s (score = %.4f)\n', cross_t, scores(cross_idx));
    fprintf('Detection latency relative to friction fault (t=8s): %.4f s\n', cross_t - 8.0);
end

%% 5. Plot
fig = figure('Name', 'PCA Anomaly Score', 'Position', [100, 100, 900, 450]);
plot(feat_time, scores, 'b-', 'LineWidth', 1.3); hold on;
yline(thresh_hi, 'r--', 'Upper threshold');
yline(thresh_lo, 'r--', 'Lower threshold');
xline(8.0,  'k:', 'Friction Fault (t=8s)');
xline(14.0, 'k:', 'Thermal Fault (t=14s)');
if ~isempty(cross_idx)
    plot(cross_t, scores(cross_idx), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
title('PCA (PC1) Anomaly Score — Single-Baseline Classifier');
xlabel('Time (s)'); ylabel('PC1 Score (standardized)'); grid on;
saveas(fig, fullfile(resultsDir, 'PCA_Anomaly_Score.png'));

disp('PCA classifier completed successfully.');
