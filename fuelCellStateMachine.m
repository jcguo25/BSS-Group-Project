function fcState = fuelCellStateMachine(P_req, soc, fcTimerState, P_reqAll, i, fc, batt)
% fuelCellStateMachine - Manages FC ON/OFF states based on power demand and battery SoC.

    % --- Battery Parameter Extraction & Power Limit Calculation ---
    % Calculate max battery discharge power [kW]
    P_battPackMax = batt.cell.capacity_Ah * batt.cell.CRateMax * ...
        batt.cell.ocv_volt_axis(1) * batt.Ns * batt.Np / 1000;      
    
    socChrgLmt = batt.socChrgLmt;       % High SoC threshold
    socDischrgLmt = batt.socDischrgLmt; % Low SoC threshold
    
    % --- Fuel Cell Parameter Extraction ---
    P_MaxEff = fc.P_MaxEff;             % Efficient power setpoint [kW]
    fcOnTimer = fcTimerState.OnTimer;   % Current ON duration [s]
    fcOffTimer = fcTimerState.OffTimer; % Current OFF duration [s]
    t_fcOnMin = fc.t_fcOnMin;           % Min runtime constraint [s]
    t_fcOffMin = fc.t_fcOffMin;         % Min downtime constraint [s]
    
    % --- Look-ahead Logic ---
    P_next = 0;
    if i < length(P_reqAll)
        P_next = P_reqAll(i+1);         % Anticipate demand for the next step
    end

    % --- State Machine Logic ---
    persistent currentState
    if isempty(currentState)
        currentState = "OFF";           % Default initial state
    end

    switch currentState
        case "ON"
            fcState = 1;
            % Condition to switch ON -> OFF:
            % 1. Min runtime met AND demand is below efficiency threshold
            % 2. OR SoC is high AND demand is below efficiency threshold
            if ((fcOnTimer > t_fcOnMin) && (P_req < P_MaxEff)) || ...
               ((soc > socChrgLmt) && (P_req < P_MaxEff)) 
                currentState = "OFF";
            end
            
        case "OFF"
            fcState = 0;
            % Condition to switch OFF -> ON:
            % 1. SoC is low (Force Charge)
            % 2. Demand > Max Efficiency point AND min downtime met
            % 3. Current or Next demand exceeds battery physical limits
            if ((P_req > 0) && (soc < socDischrgLmt)) || ...
               ((P_req > P_MaxEff) && (fcOffTimer > t_fcOffMin)) || ...
               (P_req > P_battPackMax) || ...
               (P_next > P_battPackMax)
                currentState = "ON";
            end
    end
end