function [fcOnTimer, fcOffTimer] = fcTimer(fcStatusCurrent, fcStatusPrev, fcTimerState, dt)
% fcTimer - Track the duration of fuel cell (FC) ON and OFF periods
%

    % Extract current timers
    fcOnTimer = fcTimerState.OnTimer;
    fcOffTimer = fcTimerState.OffTimer;

    % ----- State transitions -----
    % Transition: OFF -> ON
    if fcStatusCurrent == 1 && fcStatusPrev == 0
        % Save last OFF duration if non-zero
        if fcOffTimer > 0
            fcTimerState.OffHistory(end+1) = fcOffTimer;
        end
        fcOffTimer = 0;       % Reset OFF timer
        fcOnTimer = dt;       % Start new ON timer

    % Continuation: ON -> ON
    elseif fcStatusCurrent == 1 && fcStatusPrev == 1
        fcOnTimer = fcOnTimer + dt;

    % Transition: ON -> OFF
    elseif fcStatusCurrent == 0 && fcStatusPrev == 1
        % Save last ON duration if non-zero
        if fcOnTimer > 0
            fcTimerState.OnHistory(end+1) = fcOnTimer;
        end
        fcOnTimer = 0;        % Reset ON timer
        fcOffTimer = dt;      % Start new OFF timer

    % Continuation: OFF -> OFF
    elseif fcStatusCurrent == 0 && fcStatusPrev == 0
        fcOffTimer = fcOffTimer + dt;
    end

    
end
