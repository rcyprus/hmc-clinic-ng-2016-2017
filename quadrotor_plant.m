function [x_new, y] = quadrotor_plant(u, state, constants)

    % Physical constants.
    m = constants.m;
    L = constants.L;
    k = constants.k;
    b = constants.b;
    I = constants.I;
    kd = constants.kd;
    g = constants.g;
    dt = constants.dt;
    
    % state variables
    x = state(1);
    xdot = state(2:4);
    theta = state(5:7);
    thetadot = state(8:10);

    % Compute forces, torques, and accelerations.
    omega = thetadot2omega(thetadot, theta);
    a = acceleration(u, theta, xdot, m, g, k, kd);
    w_dot = angular_acceleration(u,omega, I, L, b, k);

    % Advance system state.
    omega = omega + dt*w_dot;
    thetadot = omega2thetadot(omega,theta);
    theta = theta + dt*thetadot;
    xdot = xdot + dt*a;
    x = x + dt*xdot(3);

    % Store simulation state for output.
    x_new = [x; xdot; theta; thetadot];
    y = x_new;
    % potentially change this if we need velocity as a state and acceleration as an output

end

% Function tehtadot2omega adapted from Gibiansky 
% Convert derivatives of roll, pitch, yaw to omega.
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    omega = W * thetadot;
end

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

% Adapted from http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20a%20Control.pdf

% Compute rotation matrix for a set of angles.
function R = rotation(angles)
    phi = angles(3);
    theta = angles(2);
    psi = angles(1);

    R = zeros(3);
    R(:, 1) = [
        cos(phi) * cos(theta)
        cos(theta) * sin(phi)
        - sin(theta)
    ];
    R(:, 2) = [
        cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi)
        cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
        cos(theta) * sin(psi)
    ];
    R(:, 3) = [
        sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)
        cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)
        cos(theta) * cos(psi)
    ];
end

% Function that computes thrust
% Inputs: inputs vector of current inputs and thrust coefficient
function T = thrust(k,inputs)
    T = [0; 0; k*(inputs(1) + inputs(2) + inputs(3) + inputs(4))];
end

% Function that calculates the angular acceleration
% Inputs: inputs current vector and coefficients
function w_dot = angular_acceleration(inputs,omega,I,L,b,k)
    tau = torques(L,k,b,inputs);
    w_dot = inv(I)*(tau-cross(omega,I*omega));
end

% Torques in the body frame
% Inputs: current output from each rotor (4 rotors)
function tau = torques(L,k,b,inputs)
    tau = [ L*k*(inputs(1) - inputs(3)); L*k*(inputs(2) - inputs(4)); ...
            b*(inputs(1) - inputs(2) + inputs(3) - inputs(4))];
            
end

% Convert omega to roll, pitch, and yaw
% Inputs: omega and angles
function thetadot = omega2thetadot(omega,angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
        W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetadot = inv(W)*omega;
end


