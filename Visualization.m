close all;

%% Battery Power and Soc in first 1800s

P_battDischrg = max(P_batt, 0);   % kW, discharge
P_battChrg    = min(P_batt, 0);   % kW, charge (negative)

t = cycle500km.time;
idx = t <= 1800;

figure('Name', 'Power of Charge & Discharge (first cycle)', 'NumberTitle', 'off');

% ---- Discharge area ----
p_pos = area(t(idx), P_battDischrg(idx), ...
    'FaceColor', [1.0 0.9 0.9], 'EdgeColor', 'none');
set(p_pos, 'HandleVisibility', 'off');
hold on;

% ---- Charge area ----
p_neg = area(t(idx), P_battChrg(idx), ...
    'FaceColor', [0.9 0.9 1.0], 'EdgeColor', 'none');
set(p_neg, 'HandleVisibility', 'off');

% ---- Power lines ----
plot(t(idx), P_battDischrg(idx), 'r', 'LineWidth', 1.2);
plot(t(idx), P_battChrg(idx),    'b', 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Power [kW]');
grid on;

% ---- SOC on right axis ----
yyaxis right
plot(t(idx), soc(idx) * 100, 'LineWidth', 1.5);
ylabel('SoC [%]');

legend('P_{Discharge}', 'P_{Charge}', 'SoC');

%% Battery Power and SoC through the whole run

figure('Name', 'Power of Charge & Discharge', 'NumberTitle', 'off');

% ---- Discharge area ----
p_pos = area(t, P_battDischrg, ...
    'FaceColor', [1.0 0.9 0.9], 'EdgeColor', 'none');
set(p_pos, 'HandleVisibility', 'off');
hold on;

% ---- Charge area ----
p_neg = area(t, P_battChrg, ...
    'FaceColor', [0.9 0.9 1.0], 'EdgeColor', 'none');
set(p_neg, 'HandleVisibility', 'off');

% ---- Power lines ----
plot(t, P_battDischrg, 'r', 'LineWidth', 1.2);
plot(t, P_battChrg,    'b', 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('Power [kW]');
grid on;

% ---- SOC on right axis ----
yyaxis right
plot(t, soc * 100, 'LineWidth', 1.5);
ylabel('SoC [%]');

legend('P_{Discharge}', 'P_{Charge}', 'SoC');

%% Current Profile of Battery Pack

figure('Name','Current Profile', 'NumberTitle','off');

plot(t, I_pack)
grid on;
xlabel('Time [s]');
ylabel('Current [A]');

%%
figure('Name','SOC and Power Overview','NumberTitle','off');

t = cycle500km.time;
fcOn = fcState;     % 1 = FC ON, 0 = FC OFF

fcDiff = diff([0; fcOn; 0]);
fcStartIdx = find(fcDiff == 1);
fcEndIdx   = find(fcDiff == -1) - 1;

yyaxis left
hold on;

for k = 1:length(fcStartIdx)
    xPatch = [ t(fcStartIdx(k)), t(fcEndIdx(k)), ...
               t(fcEndIdx(k)),   t(fcStartIdx(k)) ];
    yPatch = [0, 0, 100, 100];

    h_fc = patch(xPatch, yPatch, [1 0.8 0.6], ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

yyaxis right

p_pos = area(t, P_battDischrg, ...
    'FaceColor', [1 0 0], 'FaceAlpha', 0.2, ...
    'EdgeAlpha', 0.1, 'EdgeColor', 'r');
hold on;

p_neg = area(t, P_battChrg, ...
    'FaceColor', [0 0 1], 'FaceAlpha', 0.2, ...
    'EdgeAlpha', 0.1, 'EdgeColor', 'b');

ylabel('Power [kW]');
ylim([min(P_battChrg) max(P_battDischrg)]);

yyaxis left
p_soc = plot(t, soc * 100, ...
    'LineWidth', 2, 'Color', [0.0 0.0 0.6]);

ylabel('SOC [%]');
ylim([0 100]);

xlabel('Time [s]');

legend([h_fc, p_pos, p_neg, p_soc], ...
    {'Fuel Cell On', 'P_{Discharge}', 'P_{Charge}', 'SoC'}, ...
    'Location', 'best');

grid on;

