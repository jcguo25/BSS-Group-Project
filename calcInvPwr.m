%% Functions for load profile calculation

function P_req = calcInvPwr(cycle, veh, drvt)
    % Extract varibles
    v = cycle.speed;
    acc = cycle.acc;

    % ----- Vehicle parameters -----
    m_total = veh.m_total;
    g = veh.g;
    f_w = veh.f_w;
    rho_air = veh.rho_air;
    c_d = veh.c_d;
    A = veh.A;
    e_i = veh.e_i;
    r_wheel = veh.r_wheel;

    % ----- Drivetrain parameters -----
    i_g = drvt.i_g;
    eta_g = drvt.eta_g;
    regen_ratio = drvt.regen_ratio;
    eta_inv = drvt.eta_inverter;

    % ===============================
    % 2. Compute resistive forces
    % ===============================
    F_roll = calcRollingResistance(m_total, g, f_w);
    F_aero = calcAeroDrag(v, rho_air, c_d, A);

    % ===============================
    % 3. Compute wheel force & torque
    % ===============================
    [F_wheel, T_wheel] = calcWheelForceTorque(acc, veh.m_veh, veh.m_payload, ...
                                              e_i, F_roll, F_aero, r_wheel);

    % ===============================
    % 4. Compute motor speed & torque
    % ===============================
    motor_speed = calcMotorSpeed(v, r_wheel, i_g);
    T_motor = calcMotorTorque(T_wheel, i_g, eta_g);

    % ===============================
    % 5. Motor efficiency & power
    % ===============================
    eta_motor = calcMotorEfficiency(T_motor, motor_speed, drvt.eta_motor);

    P_drive = calcDrivePower(T_motor, motor_speed, eta_motor);   % motor output power
    P_regen = calcRegenPower(T_motor, motor_speed, eta_motor, regen_ratio); % regen braking

    % ===============================
    % 6. Power at inverter (battery & fuel cell side)
    % ===============================
    P_inv = P_drive ./ eta_inv + P_regen .* eta_inv;
    P_req = P_inv ./ 0.95;

end

% %% ===== Helper Functions =====


function F_roll = calcRollingResistance(m_total, g, f_w)
    F_roll = f_w * m_total * g;
end

function F_aero = calcAeroDrag(v_kph, rho_air, c_d, A)
    v = v_kph / 3.6;
    F_aero = 0.5 * rho_air * c_d * A .* v.^2;
end

function [F_wheel, T_wheel] = calcWheelForceTorque(acc, m_veh, m_payload, ...
                                                   e_i, F_roll, F_aero, r_wheel)
    % Equivalent mass including rotation
    m_eq = e_i * m_veh + m_payload;

    % Inertial force (positive: traction, negative: braking)
    F_inertia = m_eq .* acc;

    % Total wheel force (positive accel, negative decel)
    F_wheel = F_roll + F_aero + F_inertia;

    % Wheel torque
    T_wheel = F_wheel .* r_wheel;
end

function motor_speed = calcMotorSpeed(v_kph, r_wheel, i_g)
% calcMotorSpeed
% Calculate motor mechanical angular speed [rad/s]
%
% Inputs:
%   v        - vehicle speed [km/h]
%   r_wheel  - wheel rolling radius [m]
%   i_g      - gear ratio
%
% Output:
%   motor_speed - motor mechanical speed [rad/s]
    v = v_kph / 3.6;
    wheel_speed = v ./ r_wheel;      % rad/s (ω = v/r)
    motor_speed = wheel_speed .* i_g;
end

function T_motor = calcMotorTorque(T_wheel, i_g, eta_g)
% calcMotorTorque
% Calculate motor shaft torque from wheel torque
%
% Inputs:
%   T_wheel - wheel torque [Nm]
%   i_g     - gear ratio
%   eta_g   - gearbox efficiency (0~1)
%
% Output:
%   T_motor - motor shaft torque [Nm]

    T_motor = T_wheel ./ (i_g .* eta_g);
end


function eta = calcMotorEfficiency(T_motor, motor_speed, effMap)

    eff_map = effMap.efficiency_data;
    speed_axis = effMap.rpm_axis;
    torque_axis = effMap.torque_axis;

    T_motor_eff = abs(T_motor);
    motor_speed_rpm = motor_speed / (2 * pi) * 60;

    eta = interp2(speed_axis, torque_axis, eff_map, ...
    motor_speed_rpm, T_motor_eff, 'linear', NaN) ./ 100;

    eta(isnan(eta)) = 0.8;

    % eta = 0.90;

end



function P_drive = calcDrivePower(T_motor, motor_speed, eta_motor)
    % Mechanical power
    P_mech = T_motor .* motor_speed;   % [W]

    % Positive mechanical power → traction
    P_drive = max(P_mech, 0);

    % Convert to electrical power at battery
    P_drive = P_drive ./ eta_motor ./ 1000;    % P_elec = P_mech / eta, [kW]
end


function P_regen = calcRegenPower(T_motor, motor_speed, eta_motor, regen_ratio)
    % Mechanical power (negative during regen)
    P_mech = T_motor .* motor_speed;   % [W]

    % Only negative (braking mechanical power)
    P_brake = min(P_mech, 0);          % negative

    % Electrical regen power (negative = charging)
    P_regen = P_brake .* eta_motor .* regen_ratio ./ 1000; %[kW]
end
