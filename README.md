# Predictive Maintenance & Anomaly Detection for Electromechanical Actuators

A self-contained MATLAB/Simulink portfolio project that models progressive
mechanical and electrical degradation in an actuator, then detects it
automatically from the motor current signal, no manual inspection, no
hindsight.

## Why this project

Unplanned actuator failures are expensive: production stops, parts get
replaced reactively, and the failure mode is often invisible until it's
already causing downtime. This project builds a small, honest version of
what a real condition-monitoring system does: inject realistic degradation
into a simulated actuator, watch the current signal, and prove that a
statistical detector can catch the fault shortly after it starts, not
before, and not too late to matter.

## What it does

1. **Simulates a healthy actuator**, then injects two independent faults
   mid-run without stopping the simulation:
   - **Bearing wear** (mechanical friction increase, +150%) at t = 8s
   - **Stator thermal aging** (resistance increase, +80%) at t = 14s
2. **Processes the motor current signal** with a rolling RMS window and an
   FFT comparison across healthy/faulted windows.
3. **Detects the fault automatically** using a 3-sigma statistical threshold
   learned from the healthy baseline period, and reports how long it took
   to notice.
4. **Verifies the detector isn't a fluke** by re-running it across 9 fault
   severity levels and checking the detection latency trend makes physical
   sense.

## Repository structure

```
predictive-maintenance-actuator-diagnostics/
├── models/
│   ├── build_model.m           # Builds the model via script
│   ├── actuator_params.mat     # Saved model parameters
│   └── Actuator_FaultInjection_Model.slx
├── scripts/
│   ├── run_predictive_diagnostics.m   # Single-run detection
│   ├── run_severity_sweep.m           # 9-level severity sweep
│   ├── run_multi_fault_detection.m    # Catches both faults
│   └── run_pca_classifier.m           # PCA-based classifier
├── results/
│   ├── Predictive_Maintenance_Dashboard.png
│   ├── severity_sweep_results.csv
│   ├── severity_sweep_latency.png
│   ├── Multi_Fault_Detection.png
│   └── PCA_Anomaly_Score.png
└── docs/
    ├── technical_report.md
    ├── block_reference.md
    └── management_rationale.md
```

## How to run it

```matlab
addpath('models');
addpath('scripts');

build_model                    % builds model + params
run_predictive_diagnostics     % detects the fault, plots dashboard
run_severity_sweep             % re-runs across 9 fault severities
run_multi_fault_detection      % catches both faults, re-baselining
run_pca_classifier             % PCA classification (Stats & ML Toolbox)
```

See `docs/management_rationale.md` for the cost-benefit / Technology &
Innovation Management write-up connecting these results to why predictive
maintenance matters.

## Headline results

**Single-run detection (spec-nominal severity, +150% friction fault):**

| Metric | Value |
|---|---|
| Healthy baseline RMS | 3.7596 A (± 0.0021 A) |
| 3-sigma alarm threshold | 3.7669 A |
| Fault injected at | t = 8.000 s |
| Fault detected at | t = 8.024 s |
| Detection latency | 0.024 s |

**Severity sweep (9 levels, +10% to +300% friction increase):**

| Severity | Latency (s) | Peak RMS (A) |
|---|---|---|
| +10% | 0.086 | 4.05 |
| +25% | 0.054 | 4.48 |
| +50% | 0.039 | 5.17 |
| +75% | 0.032 | 5.82 |
| +100% | 0.028 | 6.43 |
| +150% | 0.024 | 7.58 |
| +200% | 0.021 | 8.61 |
| +250% | 0.019 | 9.56 |
| +300% | 0.017 | 10.42 |

Detection latency drops monotonically and peak RMS rises monotonically as
severity increases. This is a consistent, physically sensible trend across every
level tested.

## Honest notes on approach

- **No Simscape.** The spec's toolbox list includes Simscape Electrical/
  Driveline, but the actuator here is built as a lumped electrical/
  mechanical state-space equivalent (current and speed integrators wired
  from first-principles equations), not a Simscape physical network. This
  was a deliberate choice: the model is built entirely via `add_block`/
  `add_line` in a single script, and Simscape's physical-signal ports and
  solver configuration are far more fragile to get right via script than
  standard Simulink blocks. The fault dynamics (a parameter step multiplied
  into a physical term) don't require Simscape's physical-network solver to
  behave correctly.
- **No sensor noise.** The simulation is fully deterministic, which made the
  healthy-baseline RMS variance extremely small. That's good for a clean,
  repeatable demo, but it means the 3-sigma threshold is very tight, in the
  severity sweep, even a +10% friction increase was detected. A version of
  this project with realistic sensor noise added would show a genuine
  true-negative region at low severities; this version does not.
- **Only the severity sweep is friction-only.** The 9-level sweep
  (`run_severity_sweep.m`) varies friction severity only. Multi-event
  detection (catching both the friction AND thermal faults, separately, in
  one run) is handled by `run_multi_fault_detection.m`. See that script
  and the technical report's Section 6.1 for how it re-baselines after the
  first event to catch the second.
- **PCA classifier demonstrates the required method, but doesn't
  outperform RMS on this dataset.** `run_pca_classifier.m` uses the
  Statistics & ML Toolbox's `pca()` function as the spec requires, but
  because this model's features are highly collinear (no sensor noise),
  its PC1 score ends up tracking essentially the same information RMS
  already captures. See the technical report's Section 6.2 for the full
  honest assessment.
