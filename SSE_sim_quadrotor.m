%% Inputs
% simulation parameters
Ttotal = 1; % total sample time
Ts = 0.01;    % simulation timestep
n = 10;       % number of states
m = 5;        % number of inputs

% number of sensors
% vertical position, x/y/z velocity, theta/phi/psi, ang. accel.
num_sensors = [3 2 1 2 1 3 1 2 3 4];
p = sum(num_sensors);

% reference signal (desired state)
ref = zeros(n,1); ref(1) = 1;
u_const = (700^2)*ones(4,1);

% initialization of C matrix
C = zeros(p,n);
index = 1;
for state=1:n % loop through states
    for j=0:num_sensors(state)-1
        C(index+j,:) = zeros(1,n); C(index+j, state) = 1;
    end
    index = sum(num_sensors(1:state)) + 1;
end

% physical constants
M = 0.5;   % quadrotor mass (kg)
L = 0.25;  % distance from center of quadrotor to propeller (m)
k = 3e-6;  % constant, depends on blade configuration & parameters (kg m?)
b = 1e-7;  % motor drag coefficient, depends on propeller radius
           %      coefficient of drag, air density (kg m^2?)
I = diag([5e-3 5e-3 10e-3]); % moments of inertia (kg m^2) = motor mass * L^2?
kd = 0.25; % quadrotor drag coefficient (kg/s?) = 4?
g = 9.81;  % gravity (m/s^2)
constants = struct('C', C, 'm', M, 'L', L, 'k', k, 'b', b, ...
                   'I', I, 'kd', kd, 'g', g, 'dt', Ts);
clear M L k b I kd g

% control
% c = controller_new('pid');

%% Initialization and initial calculations
T = Ttotal/Ts; % total timesteps

% initial state and output
x_init = zeros(n,1); x_init(1) = 1;
y_init = C*x_init;
u_init = [u_const; constants.g];

% initialize matrices to store outputs and inputs
U = zeros(m,T);     U(:,1) = u_init;
Y = zeros(p,T);     Y(:,1) = y_init;
X = zeros(n,T);     X(:,1) = x_init;
Xhat = zeros(n,T);  Xhat(:,1) = x_init;
Xlin = zeros(n,T);  Xlin(:,1) = x_init;
Xinit = zeros(n,T); Xinit(:,1) = x_init;

% linearize equations of motion
[Ad, Bd] = linearize_quadrotor(u_init, x_init, constants);

Bu = zeros(p,T);

%% Simulation
for t=1:T-1 % t is the timestep number
    %% Plant
    u_prev = U(:,t); % grab control inputs
    x_prev = X(:,t);   % grab state x[t-1]
    xlin = Xlin(:,t);
    
    [x_new, y] = quadrotor_plant(u_prev, x_prev, constants); % calculate output y
    X(:,t+1) = x_new; % save new state x[t] in X matrix
    Y(:,t+1) = y;     % save outputs in Y matrix
                      %     add 0.5*randn(1) for noisy outputs

    % save linearized output
    Xlin(:,t+1) = Ad*xlin + Bd*u_prev;
    %X(:,t+1) = Xlin(:,t+1);
    %Y(:,t+1) = C*X(:,t+1);
    
    %% State estimator
    % create observability matrix
    CA = zeros(p*T,n);
    for i=0:T
        index = (i*p + 1);
        CA(index:index+(p-1),:) = C*(Ad^i);
    end
    
    % create matrix of control inputs
%     Bu = zeros(p,T);
%     for j=1:t
%         for i=1:j
%             Bu(:,j+1) = Bu(:,j+1) + Cd*(Ad^(j-i))*Bd*U(:,i);
%         end
%     end
    
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
    fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)
    
    Xinit(:,t+1) = x; % store initial state calculated by cvx
    
    % propagate dynamics to find previous state from initial state
    for i=0:t
        xhat = Ad*x + Bd*U(:,i+1);
        x = xhat;
    end
    Xhat(:,t+1) = xhat;
    
    %% Controller
    %[u_new, constants] = c(constants, ref, X(:,t+1));
    u_new = [u_const; constants.g];
    if t*Ts > 0.5
        u_new(1:4,1) = zeros(4,1);
    end
    U(:,t+1) = u_new;
end

%% Plot results
figure;
stem(0:Ts:Ttotal-Ts, Xhat(1,:),'DisplayName','estimated state x (height z)')
hold on
%stem(0:Ts:Ttotal-Ts, Y(1,:),'DisplayName','output y (height z)')
stem(0:Ts:Ttotal-Ts, X(1,:),'DisplayName','actual state x (height z)')
plot(0:Ts:Ttotal-Ts, Xlin(1,:),'DisplayName','linearized model for x (height z)')
xlabel('time (s)')
ylabel('height z (m)')
legend('show')


