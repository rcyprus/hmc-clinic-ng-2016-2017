% Function that calculates the acceleration a
% Inputs: inputs current vector, angles, and coefficients
function a = acceleration(inputs,angles,xdot,m,g,k,kd)
    grav = [0;0;-g];
    % TODO: figure out what the rotation function is
    R = rotation(angles);
    T = R * thrust(k,inputs);
    Fd = -kd * xdot;
    a = grav + (1/m)*T + Fd;
end 