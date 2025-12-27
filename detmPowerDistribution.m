function [P_batt, P_fc] = detmPowerDistribution(fcState, P_req, fc)
    
    %% Extract data

    P_MaxEff = fc.P_MaxEff;


    %% Power distribution strategy
    if fcState == 1
        if P_req > P_MaxEff
            P_fc = P_req;
            P_batt = 0; % No battery power needed if P_req is less than or equal to P_fcMin
        elseif P_req <= P_MaxEff
            P_fc = P_MaxEff; % Set fuel cell power to minimum if required power is less than or equal to P_fcMin
            P_batt = P_req - P_MaxEff;
        end
    elseif fcState == 0
        P_fc = 0; % No fuel cell power if the status is off
        P_batt = P_req; % All required power is supplied by the battery
    end
    
end