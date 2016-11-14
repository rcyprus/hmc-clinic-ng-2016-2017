% Function that computes thrust
% Inputs: inputs vector of current inputs and thrust coefficient
function T = thrust(k,inputs)
    T = [0; 0; k*(inputs(1) + inputs(2) + inputs(3) + inputs(4))];
end