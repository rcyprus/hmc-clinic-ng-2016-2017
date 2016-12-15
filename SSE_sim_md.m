%% Inputs
Ttotal = 1;           % total sample time
Ts = 0.05;            % simulation timestep

% reference signal (desired state)
ref = 1;

% initial state
x_init = 0.2;
xhat = x_init;
y_init = x_init;

% system
b=5; M=3;
A=-b/M;
B=1/M;
C=1;

%% Initialization and initial calculations
T = Ttotal/Ts; % total timesteps

% initialize matrices to store outputs and inputs
U = zeros(1,T);
Y = zeros(1,T);     Y(1) = y_init;
X = zeros(1,T);     X(1) = x_init;
Xhat = zeros(1,T);  Xhat(1) = xhat;
Xinit = zeros(1,T); Xinit(1) = x_init;

% initialize matrices for SSE calcs
Bu = zeros(1,T);
CA = zeros(T,1); CA(1) = C;

% determine digital state space equations
sys = ss(A,B,C,0);
sysd = c2d(sys,Ts);
Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;

% control
K = dlqr(Ad,Bd,1,0.1);
Kr = -inv(Cd*inv(Ad-1-Bd*K)*Bd);

%% Simulation
for t=1:T-1 % t is the timestep number
    %% Plant
    u = U(t+1); % grab control inputs u[t]
    x = X(t);   % grab state x[t-1]
    [x_new, y] = plant_md(Ad, Bd, Cd, u, x);
    Y(t+1) = y + 0.5*rand(1);     % save noisy y[t] output in Y matrix
    X(t+1) = x_new; % save new state x[t] in X matrix
    
    %% State estimator
    % update observability matrix
    CA(t+1) = Cd*(Ad^t);
    
    % update matrix of control inputs
    for i=0:t
        Bu(t+1) = Bu(t+1) + Cd*(Ad^(t-i))*Bd*U(i+1);
    end
    
    % run optimization to find initial state x
    YBu = Y(1:t+1) - Bu(1:t+1);
    CA_t = CA(1:t+1);
    r = 2;
    cvx_begin quiet
        variable x(1)
        minimize( sum(norms(YBu - reshape(CA_t*x,[1,t+1]), r, 2)) )
    cvx_end
    fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)
    
    Xinit(t+1) = x;
    
    % propagate dynamics to find previous state from initial state
    for i=1:t
        x_new = Ad*x + Bd*U(i+1);
        x = x_new;
    end
    xhat = x_new;
    Xhat(t+1) = xhat; % save estmimated state xhat[t] in Xhat matrix
    
    %% Controller
    u_new = Kr*ref - K*xhat; %*X(:,t+1); % calculate new control inputs
    U(t+2) = u_new; % save new inputs u[t+1] in U matrix
    
end

%% Plot results
figure;
stem(0:Ts:Ttotal-Ts, Xhat,'DisplayName','estimated state x')
hold on
stem(0:Ts:Ttotal-Ts, Y,'DisplayName','output y')
stem(0:Ts:Ttotal-Ts, X,'DisplayName','actual state x')
xlabel('time (s)')
ylabel('velocity')
legend('show')


