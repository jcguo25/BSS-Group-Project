function fcState = fuelCellStateMachine(P_req, soc, fcTimerState, P_reqAll, i, fc, batt)
% fuelCellStateMachine - Manages FC ON/OFF states based on power demand and battery SoC.

    % --- Battery Parameter Extraction & Power Limit Calculation ---
    % Calculate max battery discharge power [kW]
    P_battPackMax = batt.P_DischrgPackMax;      
    
    socChrgLmt = batt.socChrgLmt;       % High SoC threshold
    socDischrgLmt = batt.socDischrgLmt; % Low SoC threshold
    
    % --- Fuel Cell Parameter Extraction ---
    P_MaxEff = fc.P_MaxEff;             % Efficient power setpoint [kW]
    fcOnTimer = fcTimerState.OnTimer;   % Current ON duration [s]
    fcOffTimer = fcTimerState.OffTimer; % Current OFF duration [s]
    t_fcOnMin = fc.t_fcOnMin;           % Min runtime constraint [s]
    t_fcOffMin = fc.t_fcOffMin;         % Min downtime constraint [s]
    
    % --- Look-ahead Logic ---
    PreqPredictPeriod = 100;
    if i < length(P_reqAll)-PreqPredictPeriod
        P_next = P_reqAll(i:i+PreqPredictPeriod);         % Anticipate demand for the next step
    else
        P_next = P_reqAll(i:end);
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
            if (P_req < P_MaxEff) && (soc > socDischrgLmt)
                currentState = "IDLE";
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
               any(P_next > P_battPackMax)
                currentState = "ON";
            end

        case "IDLE"  
            fcState = 2;
            if ((length(P_next(P_next<P_MaxEff)) > length(PreqPredictPeriod)*0.8) && (soc > socChrgLmt) && (fcOnTimer > t_fcOnMin)) || ...
                    (soc > 0.9)
                currentState = "OFF";
            elseif ((P_req > P_MaxEff)) || ...
                    (soc < socDischrgLmt)
                currentState = "ON";
            end
    end
end