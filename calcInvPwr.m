%% Functions for load profile calculation

function P_req = calcInvPwr(cycle, veh, drvt)
% calcInvPwr - Main wrapper to calculate DC bus power demand [kW]
    
    v = cycle.speed; % [km/h]
    acc = cycle.acc; % [m/s^2]

    % 1. Compute resistive forces [N]
    F_roll = calcRollingResistance(veh.m_total, veh.g, veh.f_w);
    F_aero = calcAeroDrag(v, veh.rho_air, veh.c_d, veh.A);

    % 2. Compute wheel force [N] & torque [Nm]
    [~, T_wheel] = calcWheelForceTorque(acc, veh.m_veh, veh.m_payload, ...
                                              veh.e_i, F_roll, F_aero, veh.r_wheel);

    % 3. Compute motor kinematics & dynamics
    motor_speed = calcMotorSpeed(v, veh.r_wheel, drvt.i_g); % [rad/s]
    T_motor = calcMotorTorque(T_wheel, drvt.i_g, drvt.eta_g); % [Nm]

    % 4. Efficiency interpolation
    eta_motor = calcMotorEfficiency(T_motor, motor_speed, drvt.eta_motor); % [0~1]

    % 5. Power split (Traction vs. Regeneration) [kW]
    P_drive = calcDrivePower(T_motor, motor_speed, eta_motor);   
    P_regen = calcRegenPower(T_motor, motor_speed, eta_motor, drvt.regen_ratio); 

    % 6. Total Inverter Power [kW]
    % Note: P_regen is already negative; eta_inv is applied based on energy flow direction
    P_inv = P_drive ./ drvt.eta_inverter + P_regen .* drvt.eta_inverter;
    
    % 7. Final DC Bus Request [kW]
    % Assuming a constant 95% efficiency for distribution/cabling
    P_req = P_inv ./ 0.95; 
end

% --- Helper Function Details ---

function F_roll = calcRollingResistance(m_total, g, f_w)
    F_roll = f_w * m_total * g; % Standard rolling friction formula
end

function F_aero = calcAeroDrag(v_kph, rho_air, c_d, A)
    v = v_kph / 3.6; % Convert km/h to m/s
    F_aero = 0.5 * rho_air * c_d * A .* v.^2; 
end

function [F_wheel, T_wheel] = calcWheelForceTorque(acc, m_veh, m_payload, ...
                                                   e_i, F_roll, F_aero, r_wheel)
    % Equivalent mass accounting for rotating components (e_i > 1.0)
    m_eq = e_i * m_veh + m_payload;
    F_inertia = m_eq .* acc;
    F_wheel = F_roll + F_aero + F_inertia;
    T_wheel = F_wheel .* r_wheel; % Torque required at the wheel center
end

function motor_speed = calcMotorSpeed(v_kph, r_wheel, i_g)
    v = v_kph / 3.6; 
    wheel_speed = v ./ r_wheel;      % [rad/s]
    motor_speed = wheel_speed .* i_g; % [rad/s]
end

function T_motor = calcMotorTorque(T_wheel, i_g, eta_g)
    % Transmission of torque through gearbox: T_mot = T_whl / (i * eta)
    T_motor = T_wheel ./ (i_g .* eta_g);
end

function eta = calcMotorEfficiency(T_motor, motor_speed, effMap)
    % Interpolates efficiency from the 2D Motor Map generated previously
    eff_map = effMap.efficiency_data;
    speed_axis = effMap.rpm_axis;
    torque_axis = effMap.torque_axis;
    
    T_motor_eff = abs(T_motor); % Map uses absolute torque
    motor_speed_rpm = motor_speed / (2 * pi) * 60; % Convert rad/s to RPM
    
    eta = interp2(speed_axis, torque_axis, eff_map, ...
          motor_speed_rpm, T_motor_eff, 'linear', NaN) ./ 100;
    
    % Default efficiency if operating outside the map boundaries
    eta(isnan(eta)) = 0.8; 
end

function P_drive = calcDrivePower(T_motor, motor_speed, eta_motor)
    P_mech = T_motor .* motor_speed; % Mechanical power [W]
    P_drive = max(P_mech, 0);        % Consider only positive (traction) power
    P_drive = P_drive ./ eta_motor ./ 1000; % Electrical power input [kW]
end

function P_regen = calcRegenPower(T_motor, motor_speed, eta_motor, regen_ratio)
    P_mech = T_motor .* motor_speed; % Mechanical power [W]
    P_brake = min(P_mech, 0);        % Consider only negative (braking) power
    % Electrical power output [kW]. regen_ratio represents the % of braking done by motor
    P_regen = P_brake .* eta_motor .* regen_ratio ./ 1000; 
end