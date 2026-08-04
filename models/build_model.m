%% build_model.m
% Builds "Actuator_FaultInjection_Model.slx" entirely via script (add_block/add_line).
% No manual GUI wiring, no Simscape — a lumped electrical/mechanical state-space
% equivalent of a PMSM-driven actuator, with two independently-triggered fault
% modes injected as time-varying parameter steps. This project is self-contained:
% the model below is built fresh for this project only.
%
% Electrical:   L * di/dt   = Vin - R(t)*i - Ke*omega
% Mechanical:   J * domega/dt = Kt*i - B(t)*omega - Tload
%
% Fault A (mechanical bearing wear):     B(t) = Bnom + 1.5*Bnom * step(t-8)
% Fault B (electrical thermal aging):    R(t) = Rnom + 0.8*Rnom * step(t-14)

clear; clc; bdclose('all');

modelName = 'Actuator_FaultInjection_Model';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

%% ---- Nominal physical parameters (typical small PMSM actuator) ----
Vin_amp   = 24;      % [V] supply voltage
Rnom      = 0.8;      % [Ohm] nominal stator resistance
L         = 0.003;    % [H] stator inductance
Ke        = 0.05;     % [V*s/rad] back-EMF constant
Kt        = 0.05;     % [N*m/A] torque constant
J         = 0.0008;   % [kg*m^2] rotor + load inertia
Bnom      = 0.0004;   % [N*m*s/rad] nominal viscous friction
Tload     = 0.02;     % [N*m] constant load torque

assignin('base','Vin_amp',Vin_amp);
assignin('base','Rnom',Rnom);
assignin('base','L',L);
assignin('base','Ke',Ke);
assignin('base','Kt',Kt);
assignin('base','J',J);
assignin('base','Bnom',Bnom);
assignin('base','Tload',Tload);

%% ---- Canvas layout bookkeeping ----
x = 30; y = 30; dx = 90; dy = 80;
pos = @(x,y,w,h) [x y x+w y+h];

%% ---- Source: constant supply voltage ----
add_block('simulink/Sources/Constant', [modelName '/Vin'], ...
    'Value', 'Vin_amp', 'Position', pos(x, y, 40, 30));

%% ---- Fault A: friction step (bearing wear), +150% at t=8s ----
add_block('simulink/Sources/Step', [modelName '/Friction_Fault_Step'], ...
    'Time', '8', 'Before', '0', 'After', num2str(1.5*Bnom), ...
    'Position', pos(x, y+3*dy, 40, 30));
add_block('simulink/Sources/Constant', [modelName '/Bnom_Const'], ...
    'Value', 'Bnom', 'Position', pos(x, y+4*dy, 40, 30));
add_block('simulink/Math Operations/Add', [modelName '/B_t_Sum'], ...
    'Inputs', '++', 'Position', pos(x+dx, y+3.5*dy, 30, 40));

%% ---- Fault B: stator resistance step (thermal aging), +80% at t=14s ----
add_block('simulink/Sources/Step', [modelName '/Thermal_Fault_Step'], ...
    'Time', '14', 'Before', '0', 'After', num2str(0.8*Rnom), ...
    'Position', pos(x, y+6*dy, 40, 30));
add_block('simulink/Sources/Constant', [modelName '/Rnom_Const'], ...
    'Value', 'Rnom', 'Position', pos(x, y+7*dy, 40, 30));
add_block('simulink/Math Operations/Add', [modelName '/R_t_Sum'], ...
    'Inputs', '++', 'Position', pos(x+dx, y+6.5*dy, 30, 40));

%% ---- Electrical loop: L*di/dt = Vin - R(t)*i - Ke*omega ----
add_block('simulink/Math Operations/Product', [modelName '/R_times_i'], ...
    'Inputs', '2', 'Position', pos(x+2*dx, y+dy, 30, 30));
add_block('simulink/Math Operations/Gain', [modelName '/Ke_Gain'], ...
    'Gain', 'Ke', 'Position', pos(x+2*dx, y+2*dy, 30, 30));
add_block('simulink/Math Operations/Add', [modelName '/V_Sum'], ...
    'Inputs', '+--', 'Position', pos(x+3*dx, y+1.3*dy, 30, 40));
add_block('simulink/Math Operations/Gain', [modelName '/Inv_L'], ...
    'Gain', '1/L', 'Position', pos(x+4*dx, y+1.3*dy, 30, 30));
add_block('simulink/Continuous/Integrator', [modelName '/Current_Integrator'], ...
    'InitialCondition', '0', 'Position', pos(x+5*dx, y+1.3*dy, 30, 30));

%% ---- Mechanical loop: J*domega/dt = Kt*i - B(t)*omega - Tload ----
add_block('simulink/Math Operations/Gain', [modelName '/Kt_Gain'], ...
    'Gain', 'Kt', 'Position', pos(x+6*dx, y, 30, 30));
add_block('simulink/Math Operations/Product', [modelName '/B_times_omega'], ...
    'Inputs', '2', 'Position', pos(x+6*dx, y+3*dy, 30, 30));
add_block('simulink/Sources/Constant', [modelName '/Tload_Const'], ...
    'Value', 'Tload', 'Position', pos(x+6*dx, y+5*dy, 40, 30));
add_block('simulink/Math Operations/Add', [modelName '/T_Sum'], ...
    'Inputs', '+--', 'Position', pos(x+7*dx, y+2*dy, 30, 40));
add_block('simulink/Math Operations/Gain', [modelName '/Inv_J'], ...
    'Gain', '1/J', 'Position', pos(x+8*dx, y+2*dy, 30, 30));
add_block('simulink/Continuous/Integrator', [modelName '/Speed_Integrator'], ...
    'InitialCondition', '0', 'Position', pos(x+9*dx, y+2*dy, 30, 30));

%% ---- Logging: motor current + speed ----
add_block('simulink/Sinks/To Workspace', [modelName '/Current_Logger'], ...
    'VariableName', 'motor_current', 'SaveFormat', 'Timeseries', ...
    'Position', pos(x+6*dx, y+1.3*dy, 60, 30));
add_block('simulink/Sinks/To Workspace', [modelName '/Speed_Logger'], ...
    'VariableName', 'motor_speed', 'SaveFormat', 'Timeseries', ...
    'Position', pos(x+10*dx, y+2*dy, 60, 30));
add_block('simulink/Sinks/Scope', [modelName '/Current_Scope'], ...
    'Position', pos(x+6*dx, y+1.8*dy, 40, 30));
add_block('simulink/Sinks/Scope', [modelName '/Speed_Scope'], ...
    'Position', pos(x+10*dx, y+2.6*dy, 40, 30));

%% ---- Wiring ----
% Fault A: B(t) = Bnom + step
add_line(modelName, 'Friction_Fault_Step/1', 'B_t_Sum/1', 'autorouting', 'on');
add_line(modelName, 'Bnom_Const/1', 'B_t_Sum/2', 'autorouting', 'on');

% Fault B: R(t) = Rnom + step
add_line(modelName, 'Thermal_Fault_Step/1', 'R_t_Sum/1', 'autorouting', 'on');
add_line(modelName, 'Rnom_Const/1', 'R_t_Sum/2', 'autorouting', 'on');

% Electrical loop
add_line(modelName, 'R_t_Sum/1', 'R_times_i/1', 'autorouting', 'on');
add_line(modelName, 'Current_Integrator/1', 'R_times_i/2', 'autorouting', 'on');
add_line(modelName, 'Current_Integrator/1', 'Kt_Gain/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Integrator/1', 'Ke_Gain/1', 'autorouting', 'on');
add_line(modelName, 'Vin/1', 'V_Sum/1', 'autorouting', 'on');
add_line(modelName, 'R_times_i/1', 'V_Sum/2', 'autorouting', 'on');
add_line(modelName, 'Ke_Gain/1', 'V_Sum/3', 'autorouting', 'on');
add_line(modelName, 'V_Sum/1', 'Inv_L/1', 'autorouting', 'on');
add_line(modelName, 'Inv_L/1', 'Current_Integrator/1', 'autorouting', 'on');

% Mechanical loop
add_line(modelName, 'B_t_Sum/1', 'B_times_omega/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Integrator/1', 'B_times_omega/2', 'autorouting', 'on');
add_line(modelName, 'Kt_Gain/1', 'T_Sum/1', 'autorouting', 'on');
add_line(modelName, 'B_times_omega/1', 'T_Sum/2', 'autorouting', 'on');
add_line(modelName, 'Tload_Const/1', 'T_Sum/3', 'autorouting', 'on');
add_line(modelName, 'T_Sum/1', 'Inv_J/1', 'autorouting', 'on');
add_line(modelName, 'Inv_J/1', 'Speed_Integrator/1', 'autorouting', 'on');

% Logging
add_line(modelName, 'Current_Integrator/1', 'Current_Logger/1', 'autorouting', 'on');
add_line(modelName, 'Current_Integrator/1', 'Current_Scope/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Integrator/1', 'Speed_Logger/1', 'autorouting', 'on');
add_line(modelName, 'Speed_Integrator/1', 'Speed_Scope/1', 'autorouting', 'on');

%% ---- Solver config: 20s run per spec ----
set_param(modelName, 'StopTime', '20');
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'MaxStep', '1e-4');

%% ---- Save ----
modelsDir = fileparts(mfilename('fullpath'));
save_system(modelName, fullfile(modelsDir, [modelName '.slx']));

% Save params to a .mat file too — downstream scripts (which start with
% `clear`) load this explicitly instead of depending on the base workspace
% still holding these variables, which caused a bug where sim() failed
% because Bnom/Rnom/L/Ke/Kt/J/Tload/Vin_amp had been cleared.
save(fullfile(modelsDir, 'actuator_params.mat'), ...
    'Vin_amp', 'Rnom', 'L', 'Ke', 'Kt', 'J', 'Bnom', 'Tload');

fprintf('Model built and saved: %s.slx\n', modelName);
fprintf('Parameters saved: actuator_params.mat\n');
