function fcState = fuelCellStateMachine(P_req, soc, fcTimerState, P_reqAll, i, fc, batt)
% fuelCellStateMachine - State machine for fuel cell ON/OFF


    % Extract Datas

    P_battPackMax = batt.cell.capacity_Ah * batt.cell.CRateMax * ...
        batt.cell.ocv_volt_axis(1) * batt.Ns * batt.Np / 1000;      % [kW], 63.6kW
    socChrgLmt = batt.socChrgLmt;
    socDischrgLmt = batt.socDischrgLmt;
    
    P_MaxEff = fc.P_MaxEff;
    fcOnTimer = fcTimerState.OnTimer;
    fcOffTimer = fcTimerState.OffTimer;
    t_fcOnMin = fc.t_fcOnMin;
    t_fcOffMin = fc.t_fcOffMin;
    
    if i < length(P_reqAll)
        P_next = P_reqAll(i+1);
    end

    % Define states
    persistent currentState
    if isempty(currentState)
        currentState = "OFF";   % Initialize state
    end

    % ----- State machine -----
    switch currentState
        case "ON"

            fcState = 1;

            % TODO: Add transition condition(s) from ON -> OFF
            % currentState = "OFF"; % example transition
            if ((fcOnTimer > t_fcOnMin) && (P_req < P_MaxEff)) || ...
                    ((soc > socChrgLmt) && (P_req < P_MaxEff)) 

                currentState = "OFF";
            end

        case "OFF"

            fcState = 0;

            % TODO: Add transition condition(s) from OFF -> ON
            % currentState = "ON"; % example transition
            if ((P_req > 0) && (soc < socDischrgLmt)) || ...
                    ((P_req > P_MaxEff) && (fcOffTimer > t_fcOffMin)) || ...
                    (P_req > P_battPackMax) || ...
                    (P_next > P_battPackMax)
                
                currentState = "ON";
            end
    end

end
