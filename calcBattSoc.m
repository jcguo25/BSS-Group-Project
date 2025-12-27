function soc_new = calcBattSoc(soc, I_pack, dt, batt)
% updateBattSOC
% Coulomb counting based SOC update

%% Extract data
cellCapacity = batt.cell.capacity_Ah;
Np = batt.Np;

%% ===== 1. Pack capacity =====
Q_pack_Ah = cellCapacity * Np;

%% ===== 2. Coulomb counting =====
dQ_Ah = I_pack * dt / 3600;

soc_new = soc - dQ_Ah / Q_pack_Ah;

%% ===== 3. Saturation =====
soc_new = min(max(soc_new, 0), 1);

end
