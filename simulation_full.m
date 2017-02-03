%% Inputs
% simulation parameters
Ttotal = 1.5; % total sample time
Ts = 0.02;    % simulation timestep
n = 10;       % number of states
m = 5;        % number of inputs

% reference signal (desired state)
%ref = zeros(n,1); ref(1) = 1;

% initial state and input
x_init = zeros(n,1); x_init(1) = 1;
u_init = [(700^2)*ones(4,1); constants.g];

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

numTrials = 50;
error = zeros(n,numTrials);

% number of sensors
% vertical position, x/y/z velocity, theta/phi/psi, ang. accel.
num_sensors = ones(1,n); num_sensors(1) = numTrials;

for trial=0:numTrials
    constants.trial = trial;
    
    [U, Y, X, Xhat, Xinit] = ...
        quadrotor_simulation(Ttotal, Ts, n, m, num_sensors, x_init, u_init, constants);
    
    error(:,trial+1) = mean(abs(X - Xhat), 2);
    
    fprintf('Trial %.0f complete\n', trial)
end

%% Plot results
% figure;
% stem(0:Ts:Ttotal-Ts, Xhat(1,:),'DisplayName','estimated state x (height z)')
% hold on
% %stem(0:Ts:Ttotal-Ts, Y(1,:),'DisplayName','output y (height z)')
% stem(0:Ts:Ttotal-Ts, X(1,:),'DisplayName','actual state x (height z)')
% plot(0:Ts:Ttotal-Ts, Xlin(1,:),'DisplayName','linearized model for x (height z)')
% xlabel('time (s)')
% ylabel('height z (m)')
% legend('show')

%% Plot error
figure; hold on
bar(0:numTrials,error(1,:))
xlabel(sprintf('number of sensors compromised out of %.0f total', numTrials))
ylabel('error in height')
xlim([0 50])

