function result = simulation(tstart,tend,dt)
    
    % Physical constants.
    g = 9.81;
    m = 0.5;
    L = 0.25;
    k = 3e-6;
    b = 1e-7;
    I = diag([5e-3, 5e-3, 10e-3]);
    kd = 0.25;
    
    
    % Define simulation time vector
    if nargin < 4
        tstart = 0;
        tend = 10;
        dt = .001;
    end
    time = tstart:dt:tend;

    N = numel(time);

    % Output values vectors initialized
    xout = zeros(3,N);
    xdotout = zeros(3,N);
    thetaout = zeros(3,N);
    thetadotout = zeros(3, N);
    inputout = zeros(4, N);

    % TODO: struct for the controller
    % controller_params = struct('dt', dt, 'I', I, 'k', k, 'L', L, 'b', b, 'm', m, 'g', g);

    % Initial system state
    x = [0; 0; 0];
    xdot = zeros(3,1);
    theta = zeros(3,1);
    thetadot = zeros(3,1);

    % Looping through the simulation
    ind = 0;
    for t = time 
        ind = ind + 1;
        % built-in input
        i = input(t);

        % Compute forces, torques, and accelerations.
        omega = thetadot2omega(thetadot,theta);
        a = acceleration(i, theta,xdot,m,g,k,kd);
        w_dot = angular_acceleration(i,omega,I,L,b,k);

        % Advance system state.
        omega = omega + dt*w_dot;
        thetadot = omega2thetadot(omega,theta);
        theta = theta + dt*thetadot;
        xdot = xdot + dt*a;
        x = x + dt*xdot;

        % Store simulation state for output.
        xout(:, ind) = x;
        xdotout(:, ind) = xdot;
        thetaout(:, ind) = theta;
        thetadotout(:, ind) = thetadot;
        inputout(:, ind) = i;

    end 

    % Put all simulation variables into an output struct.
    result = struct('x', xout, 'theta', thetaout, 'vel', xdotout, ...
                    'angvel', thetadotout, 't', time, 'dt', dt, 'input', inputout); 
end

% Arbitrary test input.
function in = input(t)
    in = zeros(4, 1);
    in(:) = 700;
    in(1) = in(1) + 150;
    in(3) = in(3) + 150;
    in = in .^ 2;
end
