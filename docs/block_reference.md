# Block & Component Reference

Every block in `Actuator_FaultInjection_Model.slx`, what it does, and why
it's there. For the story of why the architecture was built this way, see
`technical_report.md`. This is the raw reference for anyone who wants to
understand or modify the model directly.

## Source blocks

| Block | Type | Parameters | Purpose |
|---|---|---|---|
| `Vin` | Constant | Value = `Vin_amp` (24V) | Fixed supply voltage driving the electrical loop |
| `Friction_Fault_Step` | Step | Time = 8, Before = 0, After = `1.5*Bnom` | Outputs the friction fault delta — 0 before t=8s, jumps to +150% of nominal friction after |
| `Bnom_Const` | Constant | Value = `Bnom` (0.0004) | Nominal (healthy) friction/damping coefficient |
| `Thermal_Fault_Step` | Step | Time = 14, Before = 0, After = `0.8*Rnom` | Outputs the resistance fault delta — 0 before t=14s, jumps to +80% of nominal resistance after |
| `Rnom_Const` | Constant | Value = `Rnom` (0.8) | Nominal (healthy) stator resistance |
| `Tload_Const` | Constant | Value = `Tload` (0.02) | Constant mechanical load torque on the actuator |

## Fault-composition blocks

| Block | Type | Parameters | Purpose |
|---|---|---|---|
| `B_t_Sum` | Add | Inputs = `++` | Composes `B(t) = Bnom + friction_delta` — the live, time-varying friction coefficient |
| `R_t_Sum` | Add | Inputs = `++` | Composes `R(t) = Rnom + thermal_delta` — the live, time-varying stator resistance |

## Electrical loop (current dynamics)

Implements `L * di/dt = Vin - R(t)*i - Ke*omega`

| Block | Type | Parameters | Purpose |
|---|---|---|---|
| `R_times_i` | Product | 2 inputs | Computes the resistive voltage drop `R(t)*i` |
| `Ke_Gain` | Gain | Gain = `Ke` | Back-EMF term `Ke*omega` |
| `V_Sum` | Add | Inputs = `+--` | Composes `Vin - R(t)*i - Ke*omega` |
| `Inv_L` | Gain | Gain = `1/L` | Divides by inductance to get `di/dt` |
| `Current_Integrator` | Integrator | Initial condition = 0 | Integrates `di/dt` to produce motor current `i` |

## Mechanical loop (speed dynamics)

Implements `J * domega/dt = Kt*i - B(t)*omega - Tload`

| Block | Type | Parameters | Purpose |
|---|---|---|---|
| `Kt_Gain` | Gain | Gain = `Kt` | Torque-producing term `Kt*i` |
| `B_times_omega` | Product | 2 inputs | Friction torque `B(t)*omega` |
| `T_Sum` | Add | Inputs = `+--` | Composes `Kt*i - B(t)*omega - Tload` |
| `Inv_J` | Gain | Gain = `1/J` | Divides by inertia to get `domega/dt` |
| `Speed_Integrator` | Integrator | Initial condition = 0 | Integrates `domega/dt` to produce motor speed `omega` |

## Logging and observation

| Block | Type | Parameters | Purpose |
|---|---|---|---|
| `Current_Logger` | To Workspace | Variable = `motor_current`, format = Timeseries | Logs current signal for post-processing in `run_predictive_diagnostics.m` |
| `Speed_Logger` | To Workspace | Variable = `motor_speed`, format = Timeseries | Logs speed signal for post-processing |
| `Current_Scope` | Scope | — | Live view of current during simulation |
| `Speed_Scope` | Scope | — | Live view of speed during simulation |

## Nominal parameter values

Defined in `build_model.m` and saved to `actuator_params.mat`:

| Parameter | Value | Meaning |
|---|---|---|
| `Vin_amp` | 24 V | Supply voltage |
| `Rnom` | 0.8 Ω | Nominal stator resistance |
| `L` | 0.003 H | Stator inductance |
| `Ke` | 0.05 V·s/rad | Back-EMF constant |
| `Kt` | 0.05 N·m/A | Torque constant |
| `J` | 0.0008 kg·m² | Rotor + load inertia |
| `Bnom` | 0.0004 N·m·s/rad | Nominal viscous friction/damping |
| `Tload` | 0.02 N·m | Constant load torque |

## Solver configuration

| Setting | Value |
|---|---|
| Solver | `ode45` |
| Stop time | 20 s |
| Max step | 1e-4 s |

Chosen to match the spec's 20-second operating cycle and to give fine
enough time resolution for the 10 kHz-equivalent resampling done in the
diagnostics scripts.
