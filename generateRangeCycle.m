function cycleOut = generateRangeCycle( ...
        timePerCycle, speedPerCycle, rangeKm, doRandomShuffle)
% generateRangeCycle
% Generate WLTC class 3b based speed-time cycle for arbitrary mileage
%
% INPUT
%   timePerCycle     : time vector of one WLTC 3b cycle [s]
%   speedPerCycle    : speed vector of one WLTC 3b cycle [km/h]
%   rangeKm          : target total mileage [km]
%   doRandomShuffle  : true / false (shuffle 3b phases per cycle)
%
% OUTPUT
%   cycleOut.time
%   cycleOut.speed
%   cycleOut.acc

    %% ===== WLTC class 3b phase definition =====
    % Adjust indices if your source WLTC data differs
    phaseIdx.low        = 1:589;
    phaseIdx.medium     = 590:1023;
    phaseIdx.high       = 1024:1477;
    phaseIdx.extraHigh  = 1478:length(timePerCycle);

    %% ===== Initialization =====
    totalRun = 0;
    cycleIdx = 0;

    cycleOut.time = [];
    cycleOut.speed = [];
    cycleOut.displace = [];

    %% ===== Main loop =====
    while totalRun < rangeKm

        % ---- Build one WLTC cycle ----
        if doRandomShuffle
            [tCycle, vCycle] = shuffleWLTC3b( ...
                timePerCycle, speedPerCycle, phaseIdx);
        else
            tCycle = timePerCycle;
            vCycle = speedPerCycle;
        end

        sCycle = calcDisplace(tCycle, vCycle);

        % ---- Append speed ----
        cycleOut.speed = [cycleOut.speed; vCycle];

        % ---- Append time ----
        timeOffset = cycleIdx * tCycle(end) + tCycle(1);
        cycleOut.time = [cycleOut.time; tCycle + timeOffset];

        % ---- Append displacement ----
        dispOffset = cycleIdx * sCycle(end) + sCycle(1);
        cycleOut.displace = [cycleOut.displace; sCycle + dispOffset];

        totalRun = cycleOut.displace(end);

        % ---- Trim if exceeding target mileage ----
        if totalRun > rangeKm
            idx = find(cycleOut.displace <= rangeKm);
            cycleOut.time = cycleOut.time(idx);
            cycleOut.speed = cycleOut.speed(idx);
            cycleOut.displace = cycleOut.displace(idx);

            [cycleOut.time, uIdx] = unique(cycleOut.time, 'stable');
            cycleOut.speed = cycleOut.speed(uIdx);
            cycleOut.displace = cycleOut.displace(uIdx);
        end

        cycleIdx = cycleIdx + 1;
    end

    %% ===== Acceleration =====
    cycleOut.acc = calcAcceleration(cycleOut.time, cycleOut.speed);

    %% ===== Remove displacement from output =====
    cycleOut = rmfield(cycleOut, 'displace');

end

%% ===== Local helper functions =====

function [tOut, vOut] = shuffleWLTC3b(tIn, vIn, phaseIdx)
% Random shuffle of WLTC class 3b phases (cycle-aligned)

    phases(1).t = tIn(phaseIdx.low);
    phases(1).v = vIn(phaseIdx.low);

    phases(2).t = tIn(phaseIdx.medium);
    phases(2).v = vIn(phaseIdx.medium);

    phases(3).t = tIn(phaseIdx.high);
    phases(3).v = vIn(phaseIdx.high);

    phases(4).t = tIn(phaseIdx.extraHigh);
    phases(4).v = vIn(phaseIdx.extraHigh);

    order = randperm(4);

    tOut = [];
    vOut = [];
    tOffset = 0;

    for i = 1:4
        p = phases(order(i));

        tPhase = p.t - p.t(1) + tOffset;

        tOut = [tOut; tPhase];
        vOut = [vOut; p.v];

        tOffset = tOut(end);
    end
end

function a = calcAcceleration(t, v_kmh)
    v = v_kmh / 3.6;
    a = gradient(v, t);
end

function s = calcDisplace(t, v_kmh)
    v_kms = v_kmh / 3600;
    s = cumtrapz(t, v_kms);
end
