close all;

RGB = orderedcolors("gem");

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
figure('Name', 'Power of Fuel Cell', 'NumberTitle', 'off');

x_fc = [t; flipud(t)];
y_fc = [zeros(size(P_fc)); flipud(P_fc)];

P_fcArea = fill(x_fc, y_fc, ...
     RGB(4,:), ...        
     'FaceAlpha', 0.3, ...
     'EdgeColor', 'none');
hold on;
% plot(t, P_fc, 'k', 'LineWidth', 1.2);
hold on;

x_disChrg = [t; flipud(t)];
y_disChrg = [P_fc; flipud(P_fc+P_battDischrg)];
P_DischrgArea = fill(x_disChrg, y_disChrg, ...
     RGB(2,:), ...        
     'FaceAlpha', 0.3, ...
     'EdgeColor', 'none');

x_chrg = [t; flipud(t)];
y_chrg = [P_fc; flipud(P_fc+P_battChrg)];
P_ChrgArea = fill(x_chrg, y_chrg, ...
     RGB(6,:), ...        
     'FaceAlpha', 0.3, ...
     'EdgeColor', 'none');

plot(t, P_fc, 'Color', RGB(4,:), 'LineWidth', 1.5);
hold on;

grid on;
xlabel('Time [s]');
ylabel('Power [kW]');
legend([P_fcArea, P_DischrgArea, P_ChrgArea], ...
    {'P_{fc}', 'P_{Discharge}', 'P_{Charge}'}, 'Location', 'best')

%% Current Profile of Battery Pack
figure('Name','Current Profile', 'NumberTitle','off');
CurrentProfile = plot(t, I_pack); % Battery pack current profile [A]
hold on;
maxDischrgCurr = batt.cell.capacity_Ah * batt.cell.CRateMaxDischrg * batt.Np;
maxChrgCurr = -batt.cell.capacity_Ah* batt.cell.CRateMaxChrg * batt.Np;

maxDischrgCurrLine = yline(maxDischrgCurr', 'LineWidth', 1, 'Color', 'r');
maxChrgCurrLine = yline(maxChrgCurr, 'LineWidth', 1, 'Color', 'b');

grid on;
xlabel('Time [s]');
ylabel('Current [A]');
legend([CurrentProfile, maxDischrgCurrLine, maxChrgCurrLine], ...
    {'Battery Current', 'Maximum Discharge Current', 'Maximum Charge Current'}, ...
    'Location','best')

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

%% Frequency Distribution of Power, Current and SoC

figure('Name','Frequency Distribution of Power, Current and SoC','NumberTitle','off');
% Calculate and plot histograms for power, current, and SoC
subplot(3, 2, 1);
histogram(P_batt, 'Normalization', 'probability', 'BinWidth', 0.5, ...
    'FaceColor', RGB(1,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('Power [kW]');
ylabel('Frequency');
title('Frequency Distribution of Battery Power');

subplot(3, 2, 2)
histogram(P_req, 'Normalization', 'probability', 'BinWidth', 0.5, ...
    'FaceColor', RGB(2,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none')
xlabel('Power [kW]');
ylabel('Frequency');
title('Frequency Distribution of Invertor Power');

subplot(3, 2, 3)
histogram(soc, 'Normalization', 'probability', 'BinWidth', 0.001, ...
    'FaceColor', RGB(3,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('State of Charge [%]');
ylabel('Frequency');
title('Frequency Distribution of State of Charge');

subplot(3, 2, 4);
histogram(I_pack, 'Normalization', 'probability', 'BinWidth', 1, ...
    'FaceColor', RGB(4,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('Current [A]');
ylabel('Frequency');
title('Frequency Distribution of Battery Current');

subplot(3, 2, 5)
histogram(P_fc, 'Normalization', 'probability', 'BinWidth', 1, ...
    'FaceColor', RGB(5,:), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('Power [kW]');
ylabel('Frequency');
title('Frequency Distribution of Fuel Cell');

%% Heat Generation
figure('Name','Heat Generation Power of Pack','NumberTitle','off');
plot(t, P_OhmicDissipation);
hold on;
grid on;
xlabel('Time [s]');
ylabel('Power [W]');