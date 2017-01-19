% Adapted from http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20a%20Control.pdf

% Create a controller based on it's name, using a look-up table.
function c = controller_new(name, Kd, Kp, Ki)
    % Use manually tuned parameters, unless arguments provide the parameters.
    if nargin == 1
        Kd = 4;
        Kp = 3;
        Ki = 5.5;
    elseif nargin == 2 || nargin > 4
        error('Incorrect number of parameters.');
    end

    if strcmpi(name, 'pd')
        c = @(state, ref, x) pd_controller(state, ref, x, Kd, Kp);
    elseif strcmpi(name, 'pid')
        c = @(state, ref, x) pid_controller(state, ref, x, Kd, Kp, Ki);
    else
        error(sprintf('Unknown controller type "%s"', name));
    end
end

% Implement a PID controller. See simulate(controller).
function [input, state] = pid_controller(state, ref, x, Kd, Kp, Ki)
    error = ref - x;
    
    % Initialize integrals to zero when it doesn't exist.
    if ~isfield(state, 'integral')
        state.integral = zeros(10, 1);
        state.error = zeros(10,1);
        state.derivative = zeros(10, 1);
    end

     % Prevent wind-up
     if max(abs(state.integral)) > 0.01
         state.integral(:) = 0;
     end

    % Compute total thrust.
    total = state.m * state.g / state.k / ... 
          (cos(x(5)) * cos(x(6)));

    % Compute error and inputs.
    err = Kd*state.derivative + Kp*error + Ki*state.integral; %%%%% - or + ???
    input = err2inputs(state, err, total);

    % Update controller state.
    state.derivative = (error - state.error)/state.dt;
    state.error = error;
    state.integral = state.integral + state.dt .* error;
end

% Given desired torques, desired total thrust, and physical parameters,
% solve for required system inputs.
function inputs = err2inputs(state, err, total)
    e1 = err(1);
    e2 = err(2);
    e3 = err(3);
    Ix = state.I(1, 1);
    Iy = state.I(2, 2);
    Iz = state.I(3, 3);
    k = state.k;
    L = state.L;
    b = state.b;

    inputs = zeros(4, 1);
    inputs(1) = total/4 -(2*b*e1*Ix + e3*Iz*k*L)/(4*b*k*L);
    inputs(2) = total/4 + e3*Iz/(4*b) - (e2*Iy)/(2*k*L);
    inputs(3) = total/4 -(-2*b*e1*Ix + e3*Iz*k*L)/(4*b*k*L);
    inputs(4) = total/4 + e3*Iz/(4*b) + (e2*Iy)/(2*k*L);
end
