%% quadrotor_simulation.m
% Inputs:
%   Ttotal      | total simulation time
%   Ts          | timestep size
%   n           | number of states
%   m           | number of inputs
%   num_sensors | number of sensors for each state
%               | i.e. [4 2 2 2 1 1 1 1 1 1]: 4 laser scanners, 
%               |      2 each of velocity sensors, 1 of all other sensors
%   x_init      | initial state (n x 1 vector)
%   u_init      | initial inputs (m x 1 vector)
%   constants   | struct containing system parameters
%               | M, L, k, b, I, kd, and g
%
% Outputs:
%   U      | control inputs (m x T matrix)
%   Y      | sensor outputs (p x T matrix)
%   Xhat   | estimated states (n x T matrix)
%   Xinit  | initial state from CVX (n x T matrix)

function [U, Y, X, Xhat, Xinit] = ...
    quadrotor_simulation(Ttotal, Ts, n, m, num_sensors, x_init, u_init, constants)
    %% Initialization and initial calculations
    T = Ttotal/Ts; % total timesteps
    trial = constants.trial;

    % initialize C matrix
    p = sum(num_sensors);
    C = zeros(p,n);
    index = 1;
    for state=1:n
        for j=0:num_sensors(state)-1
            C(index+j,:) = zeros(1,n); C(index+j, state) = 1;
        end
        index = sum(num_sensors(1:state)) + 1;
    end
    constants.C = C;
    y_init = C*x_init;

    % initialize matrices to store outputs and inputs
    U = zeros(m,T);     U(:,1) = u_init;
    Y = zeros(p,T);     Y(:,1) = y_init;
    X = zeros(n,T);     X(:,1) = x_init;
    Xhat = zeros(n,T);  Xhat(:,1) = x_init;
    Xinit = zeros(n,T); Xinit(:,1) = x_init;

    % linearize equations of motion
    [Ad, Bd] = linearize_quadrotor(u_init, x_init, constants);
    Bu = zeros(p,T);

    %% Simulation
    for t=1:T-1 % t is the timestep number
        %% Plant
        u_prev = U(:,t); % grab control inputs
        x_prev = X(:,t);   % grab state x[t-1]
        
        [x_new, y] = quadrotor_plant(u_prev, x_prev, constants); % calculate output y
        X(:,t+1) = x_new; % save new state x[t] in X matrix
        Y(:,t+1) = y;     % save outputs in Y matrix
        Y(:,t+1) = Y(:,t+1) + 0.1*randn(p,1); % add noise to all sensor outputs
        Y(1:trial,:) = zeros(trial,T); % make first laser scanner output all zeros
        
        %% State estimator
        % create observability matrix
        CA = zeros(p*T,n);
        for i=0:T
            index = (i*p + 1);
            CA(index:index+(p-1),:) = C*(Ad^i);
        end

        % create matrix of control inputs
        CA_2 = CA(1:p*(T-t),:);
        Bu(:,t+1:end) = Bu(:,t+1:end) + reshape(CA_2*Bd*u_prev,[p,T-t]);

        % run optimization to find initial state x
        CA_t = CA(1:p*(t+1),:);
        YBu = Y(:,1:t+1) - Bu(:,1:t+1);
        r = 2;
        cvx_begin quiet
            variable x(n)
            minimize( sum(norms(YBu - (reshape(CA_t*x,[p,t+1])), r, 2)) )
        cvx_end
        %fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)

        Xinit(:,t+1) = x; % store initial state calculated by cvx

        % propagate dynamics to find previous state from initial state
        for i=0:t
            xhat = Ad*x + Bd*U(:,i+1);
            x = xhat;
        end
        Xhat(:,t+1) = xhat;

        %% Inputs
        u_new = u_init;
        if t*Ts > 0.5
            u_new(1:4,1) = zeros(4,1);
        end
        U(:,t+1) = u_new;
    end
end


