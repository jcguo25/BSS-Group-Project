clear;
clc;
clear function;

%% ===== Load cycle files =====
load("cycles_wltc.mat");
load("cycles_nedc.mat");
load("cycles_epa.mat");

load("MotorEfficiencyMap.mat")

%% ===== Parameters =====
% Define parameters structure
%   veh: vehicle parameters (weight, resistance coefficents, etc.)
%   batt: battery parameters (efficiency, capacity, etc.)
%   drvt: drivertrain parameters (gear ratio, motor efficiency, invertor efficiency, etc.)
%   fc: fuel cell parameters (rated power, effciency, etc.)

% Vehicle
veh.m_body = 1600;          % kg, vehicle body
veh.m_batt = 100;           % kg, battery
veh.m_fuelCell = 150;       % kg, fuel cell
veh.m_payload = 500;        % kg, payload
veh.m_veh = veh.m_body + veh.m_batt + veh.m_fuelCell;
veh.m_total = veh.m_veh + veh.m_payload;

veh.g = 9.81;               % m/s^2
veh.rho_air = 1.225;        % kg/m^3
veh.c_d = 0.34;             % drag coefficient
veh.A = 2.65;                % frontal area [m^2]
veh.f_w = 0.012;            % rolling resistance coefficient

veh.e_i = 1.1;              % rotational mass factor
veh.r_wheel = 0.31;         % wheel radius [m]

% Drivetrain
drvt.i_g = 9.04;             % gear ratio
drvt.eta_g = 0.97;          % gearbox efficiency

drvt.eta_motor = MotorEfficiencyMap;      % motor efficiency
drvt.regen_ratio = 0.2;     % regenerative braking ratio
drvt.eta_inverter = 0.9;    % inverter efficiency

% Fuel Cell
fc.fcStatInit = 0;         % indicator for fuel working status, 1 for on and for off;
fc.t_fcOnMin = 100;         % [s]
fc.t_fcOffMin = 500;        % [s]
fc.P_MaxEff = 20;           % [kW]
fc.P_max = 130;             % [kW]


% Battery
% batt.battCap = 5;       % kWh
batt.soc_init = 0.2;    % Initial SoC of battery pack
% batt.eta_batt = 0.95;   % effciency of charging/discharging
batt.Np = 10;           % number of cells in parallel of the pack
batt.Ns = 106;          % number of cells in serise of the pack

batt.socChrgLmt = 0.8;      
batt.socDischrgLmt = 0.6;

% ===== OCV-SOC curve for NMC Li-ion cell =====
% SOC: 0 ~ 1
% OCV: [V], max voltage = 4.0 V

batt.cell.soc_axis = [ ...
    0.00
    0.05
    0.10
    0.20
    0.30
    0.40
    0.50
    0.60
    0.70
    0.80
    0.90
    0.95
    1.00 ];

batt.cell.ocv_volt_axis = [ ...
    3.00
    3.25
    3.40
    3.55
    3.62
    3.68
    3.72
    3.76
    3.80
    3.85
    3.92
    3.97
    4.00 ];

batt.cell.capacity_Ah = 2;  %capacity of a single cell, [Ah]
batt.cell.R_int = 0.79;     % Internal resistance of a cell, [mOhm]
batt.cell.CRateMax = 10;    % Maximum C-Rate of the cell

%% Calculation

% get working cycle
timePerCycle = WLTC_class_3b.Time;
speedPerCycle = WLTC_class_3b.Data;

% timePerCycle = FTP.Time;
% speedPerCycle = FTP.Data;

cycle500km = generate500kmCycle(timePerCycle, speedPerCycle);

cruise140kph = createCruiseCondtion(140);
P_req140kph = calcInvPwr(cruise140kph, veh, drvt);

cruise200kph = createCruiseCondtion(200);
P_req200kph = calcInvPwr(cruise200kph, veh, drvt);

%% main cycle for the simulation

N = length(cycle500km.time);
dt = [0; diff(cycle500km.time)];


% set up blank lists for saving results
P_req       = zeros(N,1);
P_batt      = zeros(N,1);
P_fc        = zeros(N,1);

soc       = zeros(N,1);
V_pack    = zeros(N,1);
I_pack    = zeros(N,1);

fcState      = zeros(N,1);

% ---------- Initial conditions ----------
soc(1)    = batt.soc_init;
fcState(1) = fc.fcStatInit;

fcTimerState.OnTimer = 0;
fcTimerState.OffTimer = 0;
fcTimerState.OnHistory = [];
fcTimerState.OffHistory = [];

P_req500km = calcInvPwr(cycle500km, veh, drvt);


for i = 1:N

    cycle_i.time = cycle500km.time(i);
    cycle_i.speed = cycle500km.speed(i);
    cycle_i.acc = cycle500km.acc(i);

    if i == 1
        [fcTimerState.OnTimer, fcTimerState.OffTimer] = fcTimer(fcState(i), ...
            fcState(i), fcTimerState, dt(i));
    else
        [fcTimerState.OnTimer, fcTimerState.OffTimer] = fcTimer(fcState(i), ...
            fcState(i-1), fcTimerState, dt(i));
    end

    if i > 1
        soc(i) = calcBattSoc(soc(i-1), I_pack(i-1), dt(i), batt);
    end

    % Calculate power requirements based on current cycle conditions
    P_req(i) = calcInvPwr(cycle_i, veh, drvt);
    fcState(i) = fuelCellStateMachine(P_req(i), soc(i), fcTimerState, P_req500km, i, fc, batt);
    [P_batt(i), P_fc(i)] = detmPowerDistribution(fcState(i), P_req(i), fc);
    [V_pack(i), I_pack(i)] = calcBattVI(P_batt(i), soc(i), batt);

    printProgress(i, N)

end

energyConsum = cumsum(P_req .* dt) / 3600;
avgPwr = energyConsum(end) * 3600 / cycle500km.time(end);


%% Plot & Print

fprintf('Total energy consumption over 500 km: %.2f kWh\n', energyConsum(end));
fprintf('Average power: %.2f kW\n', avgPwr)
fprintf('Power for crusing at 140kph: %.2f kW\n', P_req140kph);
fprintf('Power for crusing at 200kph: %.2f kW\n', P_req200kph);
fprintf('Run Visualization.m to plot the results')



%% Functions

function cruise = createCruiseCondtion(v_kph)
    cruise.speed = v_kph;
    cruise.acc = 0;
end

function printProgress(current, total)
    persistent lastLength
    if isempty(lastLength), lastLength = 0; end
    
    % Clear previous characters
    fprintf(repmat('\b', 1, lastLength));
    
    % Print new message and store its length
    msg = sprintf('Progress: %0.1f%%', (current/total)*100);
    fprintf('%s', msg);
    lastLength = length(msg);
    
    % Reset if finished
    if current == total
        fprintf('\n');
        lastLength = 0;
    end
end