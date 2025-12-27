function [V_pack, I_pack] = calcBattVI(P_batt, soc, batt)
% calcBattElec
% Instantaneous battery electrical behavior (no state update)


%% Exact data
soc_axis = batt.cell.soc_axis;              % [Ah]
ocv_volt_axis = batt.cell.ocv_volt_axis;    % [V]
R_cell = batt.cell.R_int;                   % [mOhm]
Ns = batt.Ns;                               % number of cells in serise
Np = batt.Np;                               % number of cells in parallel

%% ===== 1. OCV lookup =====
V_cell_ocv = interp1( ...
    soc_axis, ...
    ocv_volt_axis, ...
    soc, 'linear', 'extrap');

V_pack_ocv = V_cell_ocv * Ns;   % [V]

%% ===== 2. Equivalent parameters =====
R_pack = R_cell * Ns / Np / 1000;   %[Ohm]

%% ===== 3. Solve current from power balance =====
P_W = P_batt * 1000;    % [W]

a = R_pack;
b = -V_pack_ocv;
c = P_W;

disc = b^2 - 4*a*c;
if disc < 0
    disc = 0;
end

I_pack = (-b - sqrt(disc)) / (2*a);

%% ===== 4. Terminal voltage =====
V_pack = V_pack_ocv - I_pack * R_pack;

end
