% Function that calculates the angular acceleration
% Inputs: inputs current vector and coefficients
function w_dot = angular_acceleration(inputs,omega,I,L,b,k)
    tau = torques(L,k,b,inputs);
    w_dot = inv(I)*(tau-cross(omega,I*omega));
end