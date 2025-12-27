close all;

%% Battery Power and Soc in first 1800s (Zoomed-in View)
% Extracting discharge (positive) and charge (negative) components
P_battDischrg = max(P_batt, 0);   % Battery discharge power [kW]
P_battChrg    = min(P_batt, 0);   % Battery charge power [kW]
t = cycle500km.time;             % Simulation time vector [s]

% Filter data for the first 1800 seconds (approx. one cycle)
idx = t <= 1800;

figure('Name', 'Power of Charge & Discharge (first cycle)', 'NumberTitle', 'off');

% ---- Discharge Visualization ----
% Create a light red shaded area to represent energy leaving the battery
p_pos = area(t(idx), P_battDischrg(idx), ...
    'FaceColor', [1.0 0.9 0.9], 'EdgeColor', 'none');
set(p_pos, 'HandleVisibility', 'off');
hold on;

% ---- Charge Visualization ----
% Create a light blue shaded area to represent energy entering the battery (Regen/FC)
p_neg = area(t(idx), P_battChrg(idx), ...
    'FaceColor', [0.9 0.9 1.0], 'EdgeColor', 'none');
set(p_neg, 'HandleVisibility', 'off');

% ---- Power Trendlines ----
plot(t(idx), P_battDischrg(idx), 'r', 'LineWidth', 1.2); % Discharge line [kW]
plot(t(idx), P_battChrg(idx),    'b', 'LineWidth', 1.2); % Charge line [kW]
xlabel('Time [s]');
ylabel('Power [kW]');
grid on;

% ---- State of Charge (SoC) Plotting ----
% Map SoC to the right Y-axis for scale clarity
yyaxis right
plot(t(idx), soc(idx) * 100, 'LineWidth', 1.5); % SoC trend [%]
ylabel('SoC [%]');
legend('P_{Discharge}', 'P_{Charge}', 'SoC');

%% Battery Power and SoC through the whole run (Full 500km View)
% This section mirrors the above logic but applies it to the entire mission duration
figure('Name', 'Power of Charge & Discharge', 'NumberTitle', 'off');

p_pos = area(t, P_battDischrg, 'FaceColor', [1.0 0.9 0.9], 'EdgeColor', 'none');
set(p_pos, 'HandleVisibility', 'off');
hold on;
p_neg = area(t, P_battChrg, 'FaceColor', [0.9 0.9 1.0], 'EdgeColor', 'none');
set(p_neg, 'HandleVisibility', 'off');

plot(t, P_battDischrg, 'r', 'LineWidth', 1.2);
plot(t, P_battChrg,    'b', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Power [kW]');
grid on;

yyaxis right
plot(t, soc * 100, 'LineWidth', 1.5);
ylabel('SoC [%]');
legend('P_{Discharge}', 'P_{Charge}', 'SoC');

%% Current Profile of Battery Pack
figure('Name','Current Profile', 'NumberTitle','off');
plot(t, I_pack) % Battery pack current profile [A]
grid on;
xlabel('Time [s]');
ylabel('Current [A]');

%% Integrated Overview: SoC and Fuel Cell Operational Logic
figure('Name','SOC and Power Overview','NumberTitle','off');
t = cycle500km.time;
fcOn = fcState;     % Digital status: 1 = FC Active, 0 = FC Standby

% Edge Detection: Identify start/stop timestamps for the Fuel Cell
fcDiff = diff([0; fcOn; 0]);
fcStartIdx = find(fcDiff == 1);
fcEndIdx   = find(fcDiff == -1) - 1;

yyaxis left
hold on;

% ---- Fuel Cell Activity Background ----
% Create shaded vertical patches (Orange) to visualize when the FC is running
for k = 1:length(fcStartIdx)
    xPatch = [ t(fcStartIdx(k)), t(fcEndIdx(k)), ...
               t(fcEndIdx(k)),   t(fcStartIdx(k)) ];
    yPatch = [0, 0, 100, 100]; % Overlay patch across full SoC height
    h_fc = patch(xPatch, yPatch, [1 0.8 0.6], ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

% ---- Power Distribution Overlay ----
yyaxis right
p_pos = area(t, P_battDischrg, 'FaceColor', [1 0 0], 'FaceAlpha', 0.2, ...
    'EdgeAlpha', 0.1, 'EdgeColor', 'r'); % Battery discharge [kW]
hold on;
p_neg = area(t, P_battChrg, 'FaceColor', [0 0 1], 'FaceAlpha', 0.2, ...
    'EdgeAlpha', 0.1, 'EdgeColor', 'b');    % Battery charge [kW]
ylabel('Power [kW]');
ylim([min(P_battChrg) max(P_battDischrg)]);

% ---- Main SoC Trajectory ----
yyaxis left
p_soc = plot(t, soc * 100, 'LineWidth', 2, 'Color', [0.0 0.0 0.6]); % Main SoC line [%]
ylabel('SOC [%]');
ylim([0 100]);
xlabel('Time [s]');
legend([h_fc, p_pos, p_neg, p_soc], ...
    {'Fuel Cell On', 'P_{Discharge}', 'P_{Charge}', 'SoC'}, ...
    'Location', 'best');
grid on;