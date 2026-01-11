function [P_batt, P_fc] = detmPowerDistribution(fcState, P_req, fc, batt)
    
    %% Extract data

    P_fcMaxEff = fc.P_MaxEff;
    P_fcMax = fc.P_max;
    P_BattChrgMax = batt.P_ChrgPackMax;
    P_BattDischrgMax = batt.P_DischrgPackMax;

    %% Power distribution strategy
    if fcState == 1
        if P_req <= P_fcMaxEff
            P_fc = P_fcMaxEff; % Set fuel cell power to minimum if required power is less than or equal to P_fcMin
            P_batt = P_req - P_fc;    % negative, charging
            if P_batt < -P_BattChrgMax
                P_batt = -P_BattChrgMax; % Limit charging power to maximum charging capacity
                P_fc = P_req - P_batt;
            end
        elseif P_req > P_fcMaxEff
            P_fc = P_req;
            P_batt = 0; % No battery power needed if P_req is less than or equal to P_fcMin
            if P_fc > P_fcMax
                P_fc = P_fcMax;
                P_batt = P_req - P_fc;
                if P_batt > P_BattDischrgMax
                    P_batt = P_BattDischrgMax; % Limit discharge power to maximum discharge capacity
                    fprintf("exceed maximum system power\n")
                end
            end
        end
    elseif fcState == 0
        P_fc = 0; % No fuel cell power if the status is off
        P_batt = P_req; % All required power is supplied by the battery
        if P_batt > P_BattDischrgMax
            P_batt = P_BattDischrgMax; % Limit battery power to maximum discharge capacity
        elseif P_batt < -P_BattChrgMax
            P_batt = -P_BattChrgMax;
        end
    end
    
end