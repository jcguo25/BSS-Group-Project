clear; clc;
load('effmap/efficiency_map_all.mat');  % Load raw experimental efficiency data

% --- Data Extraction ---
X_raw = eff_data.rpm;      % Motor speed data points
Y_raw = eff_data.torque;   % Motor torque data points
Z_raw = eff_data.eta;      % Motor efficiency data points (%)

% === Step 1: Data Augmentation & Virtual Point Injection ===
% Purpose: Add boundary points to guide interpolation at low speeds/torques
X_zero_point = 1;
Y_zero_point = 1;
Z_zero_point = 70;         % Define a virtual low efficiency at near-zero state

X_lowTorque = 1:1:16000;   % Define a speed range at zero torque
Y_lowTorque = zeros([1, length(X_lowTorque)]);
% Linearly interpolate efficiency for the low-torque boundary (80% to 98%)
Z_lowTorque = [linspace(80, 94, length(X_lowTorque)*0.125), linspace(94, 98, length(X_lowTorque)*0.875)];

% Consolidate raw and virtual data for robust surface fitting
X_raw_ext = [X_raw, X_zero_point, X_lowTorque];
Y_raw_ext = [Y_raw, Y_zero_point, Y_lowTorque];
Z_raw_ext = [Z_raw, Z_zero_point, Z_lowTorque];

% --- Visualization: Raw Data Scatter ---
figure('Name', 'Motor Efficiency - Raw Points', 'Color', 'w');
scatter3(X_raw, Y_raw, Z_raw, 100, Z_raw, 'filled', 'MarkerEdgeColor', 'k');
colormap(jet);
cbar1 = colorbar('Location', 'EastOutside');
ylabel(cbar1, 'Efficiency (%)');
title('Raw Efficiency Data Points (3D View)');
xlabel('Speed (rpm)'); ylabel('Torque (Nm)'); zlabel('Efficiency (%)');
grid on; view(3); axis tight;

% === Step 2: Grid Generation, Interpolation & Smoothing ===

% 2.1 Define Grid Resolution
X_max = max(X_raw) * 1.1;  % Set upper speed bound with 10% margin
Y_max = max(Y_raw) * 1.1;  % Set upper torque bound with 10% margin
Map_resolution = 800;      % Define grid density
X_map = linspace(0, X_max, Map_resolution);
Y_map = linspace(0, Y_max, Map_resolution);
[X_mesh, Y_mesh] = meshgrid(X_map, Y_map);

% 2.2 Cubic Interpolation
% Generates a continuous surface from scattered raw data points
Z_map = griddata(X_raw_ext, Y_raw_ext, Z_raw_ext, X_mesh, Y_mesh, 'cubic');

% 2.3 Boundary Detection (Alpha Shape)
% Purpose: Identify the feasible operating region of the motor to mask invalid extrapolation
alpha_value = 800; 
shp = alphaShape(X_raw_ext', Y_raw_ext', alpha_value);
IN = inShape(shp, X_mesh, Y_mesh); % Logic mask: 1 if within motor operating envelope

% 2.4 Gaussian Smoothing
% Purpose: Remove numerical noise and sharp gradients in the efficiency map
sigma = 3; 
Z_temp = Z_map;
Z_temp(isnan(Z_temp)) = 0;           % Temporal fill for the filter
Z_map_smooth = imgaussfilt(Z_temp, sigma);

% 2.5 Apply Mask & Clipping
% Restore NaN to areas outside the motor's physical operating envelope
Z_final_map = NaN(size(Z_map_smooth)); 
Z_final_map(IN) = Z_map_smooth(IN); 

% Efficiency Constraint (Clipping physical limits)
eta_min_limit = 70;
eta_max_limit = 98;
Z_final_map(Z_final_map < eta_min_limit) = eta_min_limit; % Lower bound efficiency
Z_final_map(Z_final_map > eta_max_limit) = eta_max_limit; % Upper bound efficiency (anti-overshoot)

% 2.7 Re-insertion of Original Data
% Ensure the map matches exact raw experimental values at specific coordinates
for i = 1:length(X_raw_ext)
    [~, ix] = min(abs(X_map - X_raw_ext(i)));
    [~, iy] = min(abs(Y_map - Y_raw_ext(i)));
    clamped_raw_eta = max(eta_min_limit, min(eta_max_limit, Z_raw_ext(i)));
    Z_final_map(iy, ix) = clamped_raw_eta; 
end

% --- Save Processed Map ---
MotorEfficiencyMap.efficiency_data = Z_final_map; 
MotorEfficiencyMap.rpm_axis = X_map;
MotorEfficiencyMap.torque_axis = Y_map;
save('MotorEfficiencyMap.mat', 'MotorEfficiencyMap');

% === Step 3: Final Visualization (Contour Map) ===
figure('Name', 'Motor Efficiency Map (Alpha Shape Bounded)', 'Color', 'w');
% Plot filled efficiency contours
[C, h] = contourf(X_mesh, Y_mesh, Z_final_map);
colormap(jet);
cbar3 = colorbar('Location', 'EastOutside');
ylabel(cbar3, 'Efficiency (%)');
hold on;

% Plot Alpha Shape Envelope (Boundary of motor operation)
[bf, p] = boundaryFacets(shp);
plot(p(bf, 1), p(bf, 2), 'k--', 'LineWidth', 1.5); 

% Plot standard efficiency isolines for readability
levels = [80, 85, 88, 90, 92, 93, 94, 95, 96, 97];
[C2, h2] = contour(X_mesh, Y_mesh, Z_final_map, levels, 'k-', 'LineWidth', 1);
clabel(C2, h2, 'FontSize', 9, 'Color', 'k', 'Rotation', 0, 'LabelSpacing', 200);

xlabel('Speed (rpm)'); ylabel('Torque (Nm)');
title('Final Processed Motor Efficiency Map');
grid on;