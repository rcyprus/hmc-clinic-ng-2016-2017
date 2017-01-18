%% Inputs
% simulation parameters
Ttotal = 0.5; % total sample time
Ts = 0.05;    % simulation timestep
p = 10;       % number of sensors
n = 10;       % number of states
m = 4;        % number of inputs

% reference signal (desired state)
% ref = zeros(n,1); ref(1) = 1;
ref = 0.1*randn(n,1); ref(1) = 1;

% initial state
x_init = zeros(n,1); xhat = x_init;
y_init = x_init;

% physical constants
M = 0.5;   % quadrotor mass (kg)
L = 0.25;  % distance from center of quadrotor to propeller (m)
k = 3e-6;  % constant, depends on blade configuration & parameters (kg m?)
b = 1e-7;  % motor drag coefficient, depends on propeller radius
           %      coefficient of drag, air density (kg m^2?)
I = diag([5e-3 5e-3 10e-3]); % moments of inertia (kg m^2) = motor mass * L^2?
kd = 0.25; % quadrotor drag coefficient (kg/s?) = 4?
g = 9.81;  % gravity (m/s^2)
constants = struct('m', M, 'L', L, 'k', k, 'b', b, ...
                   'I', I, 'kd', kd, 'g', g, 'dt', Ts);
clear M L k b I kd g

% control
c = controller_new('pid');

%% Initialization and initial calculations
T = Ttotal/Ts; % total timesteps

% initialize matrices to store outputs and inputs
U = zeros(m,T);    U(:,1:2) = 0.1*randn(m,2);
Y = zeros(p,T);    Y(:,1) = y_init;
X = zeros(n,T);    X(:,1) = x_init;
Xhat = zeros(n,T); Xhat(:,1) = xhat;

%% Simulation
for t=1:T-1 % t is the timestep number
    %% Plant
    u = U(:,t+1); % grab control inputs
    x = X(:,t);   % grab state x[t-1]
    
    [x_new, y] = quadrotor_plant(u, x, constants); % calculate output y
    X(:,t+1) = x_new; % save new state x[t] in X matrix
    Y(:,t+1) = y;     % save outputs in Y matrix
                      %     add 0.5*randn(1) for noisy outputs
    
%     %% State estimator
%     % Linearize state space equations
%     [Ad, Bd, Cd] = linearize_quadrotor(x_new, u, constants);
%     
%     % create observability matrix
%     CA = zeros(p*T,n);
%     for i=0:t
%         index = (i*p + 1);
%         CA(index:index+(p-1),:) = Cd*(Ad^i);
%     end
%     
%     % create matrix of control inputs
%     Bu = zeros(p,T);
%     for j=1:t
%         for i=0:t
%             Bu(:,j+1) = Bu(:,j+1) + Cd*(Ad^(j-i))*Bd*U(:,i+1);
%         end
%     end
%     
%     % run optimization to find initial state x
%     YBu = Y(:,1:t+1) + Bu(:,1:t+1);
%     CA_t = CA(1:(t+1)*p,:);
%     r = 2;
%     cvx_begin quiet
%         variable x(n)
%         minimize( sum(norms(YBu + reshape(CA_t*x,[p,t+1]), r, 2)) )
%     cvx_end
%     fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)
%     
%     % propagate dynamics to find previous state from initial state
%     for i=0:t
%         x_new = Ad*x + Bd*U(:,i+1);
%         x = x_new;
%     end
%     X(:,t+1) = x_new;
     xhat = x_new;
    
    %% Controller
    [u_new, constants] = c(constants, ref, xhat);
    U(:,t+2) = u_new;
end


