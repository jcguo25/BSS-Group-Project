function cycle500km = generate500kmCycle(timePerCycle, speedPerCycle)
% generate500kmCycle
% Generate vehicle speed-time-displacement curves over a 500km range
%
% Inputs:
%   timePerCycle  - time vector for a single cycle [s]
%   speedPerCycle - speed vector for a single cycle [km/h]
%
% Outputs:
%   cycle500km - structure containing:
%       .time       - time vector for 500km range [s]
%       .speed      - speed vector for 500km range [km/h]
%       .displace   - cumulative displacement for 500km range [km]
%       .acc        - acceleration vector for 500km range [m/s^2]

    % Calculate displacement for a single cycle
    displacePerCycle = calcDisplace(timePerCycle, speedPerCycle);

    % Initialize
    targetRange = 500; % km
    totalRun = 0;
    cycle_idx = 0;
    cycle500km.time = [];
    cycle500km.speed = [];
    cycle500km.displace = [];

    % Repeat cycles until target range is reached
    while totalRun < targetRange
        % Append speed
        cycle500km.speed = [cycle500km.speed; speedPerCycle];

        % Append time with offset
        timeOffset = cycle_idx * timePerCycle(end) + timePerCycle(1);
        cycle500km.time = [cycle500km.time; timePerCycle + timeOffset];

        % Append displacement with offset
        displaceOffset = cycle_idx * displacePerCycle(end) + displacePerCycle(1);
        cycle500km.displace = [cycle500km.displace; displacePerCycle + displaceOffset];

        % Update total run
        totalRun = cycle500km.displace(end);

        % Trim if exceeds target range
        if totalRun > targetRange
            idxTrim = find(cycle500km.displace <= targetRange);
            cycle500km.displace = cycle500km.displace(idxTrim);
            cycle500km.time = cycle500km.time(idxTrim);
            cycle500km.speed = cycle500km.speed(idxTrim);
            [cycle500km.time, uniqIdx] = unique(cycle500km.time, 'stable');
            cycle500km.speed = cycle500km.speed(uniqIdx);
            cycle500km.displace = cycle500km.displace(uniqIdx);
        end

        % Increment cycle
        cycle_idx = cycle_idx + 1;
    end

    cycle500km.acc = calcAcceleration(cycle500km.time, cycle500km.speed);
    
end


%% ===== Helper Functions =====

function a = calcAcceleration(t, v_kmh)
    v = v_kmh / 3.6;
    a = gradient(v, t);
end

function s = calcDisplace(t, v_kmh)
    v_kms = v_kmh / 3600;
    s = cumtrapz(t, v_kms);
end
