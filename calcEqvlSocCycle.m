function eqvlSocCycles = calcEqvlSocCycle(soc)
    eqvlSocCycles = sum(abs(diff(soc))) / 2;
end