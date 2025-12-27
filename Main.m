clear;
clc;
clear function;

%% ===== Load cycle files =====
% Loading standard driving cycles and pre-processed motor efficiency data
load("cycles_wltc.mat");        % Worldwide Harmonized Light Vehicles Test Cycle
load("cycles_nedc.mat");        % New European Driving Cycle
load("cycles_epa.mat");         % US Environmental Protection Agency Cycles
load("MotorEfficiencyMap.mat")  % Motor efficiency look-up table [speed, torque]

%% ===== Parameters =====
% Structure Definitions:
%   veh:  Vehicle physical properties and coefficients
%   batt: Battery pack configuration and electrochemical properties
%   drvt: Drivetrain component efficiencies and ratios
%   fc:   Fuel Cell system constraints and performance targets

% --- Vehicle Parameters ---
veh.m_body = 1600;              % Vehicle body mass [kg]
veh.m_batt = 100;               % Battery pack mass [kg]
veh.m_fuelCell = 150;           % Fuel cell system mass [kg]
veh.m_payload = 500;            % Additional payload/passenger mass [kg]
veh.m_veh = veh.m_body + veh.m_batt + veh.m_fuelCell; % Net vehicle mass [kg]
veh.m_total = veh.m_veh + veh.m_payload;             % Gross vehicle mass (GVM) [kg]
veh.g = 9.81;                   % Gravitational acceleration [m/s^2]
veh.rho_air = 1.225;            % Air density [kg/m^3]
veh.c_d = 0.34;                 % Aerodynamic drag coefficient [-]
veh.A = 2.65;                   % Vehicle frontal area [m^2]
veh.f_w = 0.012;                % Rolling resistance coefficient [-]
veh.e_i = 1.1;                  % Equivalent mass factor for rotational inertia [-]
veh.r_wheel = 0.31;             % Effective wheel rolling radius [m]

% --- Drivetrain Parameters ---
drvt.i_g = 9.04;                % Transmission/Gear reduction ratio [-]
drvt.eta_g = 0.97;              % Gearbox mechanical efficiency [-]
drvt.eta_motor = MotorEfficiencyMap; % Motor efficiency data structure
drvt.regen_ratio = 0.2;         % Proportion of braking energy recovered by motor [-]
drvt.eta_inverter = 0.9;        % Power electronics (Inverter) efficiency [-]

% --- Fuel Cell (FC) Parameters ---
fc.fcStatInit = 0;              % Initial FC status (1: ON, 0: OFF)
fc.t_fcOnMin = 100;             % Minimum required continuous ON time [s]
fc.t_fcOffMin = 500;            % Minimum required continuous OFF time [s]
fc.P_MaxEff = 20;               % Power output at peak stack efficiency [kW]
fc.P_max = 130;                 % Maximum rated power of the FC stack [kW]

% --- Battery Parameters ---
batt.soc_init = 0.2;            % Initial State of Charge (0 to 1)
batt.Np = 10;                   % Number of parallel cell strings in the pack [-]
batt.Ns = 106;                  % Number of series cells in the pack [-]
batt.socChrgLmt = 0.8;          % SoC upper limit (FC stops charging battery) [-]
batt.socDischrgLmt = 0.6;       % SoC lower limit (FC starts to support/charge) [-]

% --- OCV-SOC curve for NMC Li-ion cell ---
% Battery Open Circuit Voltage [V] relative to State of Charge (0~1)
batt.cell.soc_axis = [0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95, 1.00];
batt.cell.ocv_volt_axis = [3.00, 3.25, 3.40, 3.55, 3.62, 3.68, 3.72, 3.76, 3.80, 3.85, 3.92, 3.97, 4.00];
batt.cell.capacity_Ah = 2;      % Capacity of a single battery cell [Ah]
batt.cell.R_int = 0.79;         % Single cell internal resistance [mOhm]
batt.cell.CRateMax = 10;        % Maximum allowable C-rate for discharging [-]

%% Calculation
% Select the base driving cycle (e.g., WLTC Class 3b)
timePerCycle = WLTC_class_3b.Time;
speedPerCycle = WLTC_class_3b.Data;

% Synthesize a long-distance mission (500 km range)
cycle500km = generate500kmCycle(timePerCycle, speedPerCycle);

% Performance Benchmark: Calculate steady-state cruising power
cruise140kph = createCruiseCondtion(140);
P_req140kph = calcInvPwr(cruise140kph, veh, drvt); % [kW] at 140 km/h
cruise200kph = createCruiseCondtion(200);
P_req200kph = calcInvPwr(cruise200kph, veh, drvt); % [kW] at 200 km/h

%% Main Simulation Loop
% Iterate through each time step of the mission profile
N = length(cycle500km.time);
dt = [0; diff(cycle500km.time)]; % Time step intervals [s]

% Initialize result vectors
P_req     = zeros(N,1);         % Total DC bus power demand [kW]
P_batt    = zeros(N,1);         % Battery output power [kW]
P_fc      = zeros(N,1);         % Fuel cell output power [kW]
soc       = zeros(N,1);         % Battery State of Charge [-]
V_pack    = zeros(N,1);         % Battery pack terminal voltage [V]
I_pack    = zeros(N,1);         % Battery pack current [A]
fcState   = zeros(N,1);         % Fuel cell state (ON/OFF) [-]

% Set initial conditions
soc(1)    = batt.soc_init;
fcState(1) = fc.fcStatInit;
fcTimerState.OnTimer = 0;
fcTimerState.OffTimer = 0;
fcTimerState.OnHistory = [];
fcTimerState.OffHistory = [];

% Pre-calculate the global power request for the entire mission [kW]
P_req500km = calcInvPwr(cycle500km, veh, drvt);

% Execute time-step simulation
for i = 1:N
    % Extract instantaneous cycle state
    cycle_i.time = cycle500km.time(i);
    cycle_i.speed = cycle500km.speed(i);
    cycle_i.acc = cycle500km.acc(i);
    
    % Update Fuel Cell State Timer (tracks how long FC has been in current state)
    if i == 1
        [fcTimerState.OnTimer, fcTimerState.OffTimer] = fcTimer(fcState(i), ...
            fcState(i), fcTimerState, dt(i));
    else
        [fcTimerState.OnTimer, fcTimerState.OffTimer] = fcTimer(fcState(i), ...
            fcState(i-1), fcTimerState, dt(i));
    end
    
    % Update Battery SoC based on previous step current integration
    if i > 1
        soc(i) = calcBattSoc(soc(i-1), I_pack(i-1), dt(i), batt);
    end
    
    % 1. Calculate current power demand [kW]
    P_req(i) = P_req500km(i);
    
    % 2. Energy Management Strategy (EMS): Determine if FC should be ON or OFF
    fcState(i) = fuelCellStateMachine(P_req(i), soc(i), fcTimerState, P_req500km, i, fc, batt);
    
    % 3. Power Split: Distribute requested power between battery and fuel cell
    [P_batt(i), P_fc(i)] = detmPowerDistribution(fcState(i), P_req(i), fc);
    
    % 4. Battery Dynamics: Calculate resulting voltage and current
    [V_pack(i), I_pack(i)] = calcBattVI(P_batt(i), soc(i), batt);
    
    % Display simulation progress in command window
    printProgress(i, N)
end

% Post-processing: Calculate cumulative energy and average power
energyConsum = cumsum(P_req .* dt) / 3600; % [kWh]
avgPwr = energyConsum(end) * 3600 / cycle500km.time(end); % [kW]

%% Plot & Results Output
fprintf('Total energy consumption over 500 km: %.2f kWh\n', energyConsum(end));
fprintf('Average power: %.2f kW\n', avgPwr)
fprintf('Power for cruising at 140kph: %.2f kW\n', P_req140kph);
fprintf('Power for cruising at 200kph: %.2f kW\n', P_req200kph);
fprintf('Run Visualization.m to plot the results\n')

%% ===== Helper Functions =====

function cruise = createCruiseCondtion(v_kph)
% createCruiseCondtion - Create a steady-speed driving state
% Input:  v_kph (Speed in km/h)
% Output: Struct containing constant speed and zero acceleration
    cruise.speed = v_kph;
    cruise.acc = 0;
end

function printProgress(current, total)
% printProgress - Displays a dynamic percentage progress bar in the console
    persistent lastLength
    if isempty(lastLength), lastLength = 0; end
    
    % Clear previous line output
    fprintf(repmat('\b', 1, lastLength));
    
    % Calculate and print current percentage
    msg = sprintf('Progress: %0.1f%%', (current/total)*100);
    fprintf('%s', msg);
    lastLength = length(msg);
    
    % Reset persistent variable at completion
    if current == total
        fprintf('\n');
        lastLength = 0;
    end
end