% Torques in the body frame
% Inputs: current output from each rotor (4 rotors)
function tau = torques(L,k,b,inputs)
    tau = [ L*k*(inputs(1) - inputs(3)); L*k*(inputs(2) - inputs(4)); ...
            b*(inputs(1) - inputs(2) + inputs(3) - inputs(4))];
            
end  


