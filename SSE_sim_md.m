%% Inputs
Ttotal = 1; % total sample time
Ts = 0.05;    % simulation timestep
p = 1;        % number of sensors
n = 1;        % number of states
m = 1;        % number of inputs

% reference signal (desired state)
ref = zeros(m,1); ref(1) = 1;

% initial state
%x_init = zeros(n,1); %x_init(1) = randn(1);
xhat = x_init;
y_init = zeros(p,1); y_init(1) = x_init(1);

% system
% c=1; M=1; k=1;
% A=[0 1; -k/M -c/M];
% B=[0 1/M]';
% C=[1 0];
b=5; M=3;
A=-b/M;
B=1/M;
C=1;

%% Initialization and initial calculations
T = Ttotal/Ts; % total timesteps

% initialize matrices to store outputs and inputs
U = zeros(m,T);
Y = zeros(p,T);     Y(:,1) = y_init;
X = zeros(n,T);     X(:,1) = x_init;
Xhat = zeros(n,T);  Xhat(:,1) = xhat;
Xinit = zeros(n,T); Xinit(:,1) = x_init;

% initialize matrices for SSE calcs
Bu = zeros(p,T);
CA = zeros(p*T,n); CA(1:p,:) = C;

% determine digital state space equations
sys = ss(A,B,C,0);
sysd = c2d(sys,Ts);
Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;

% control
K = dlqr(Ad,Bd,eye(n),0.1*eye(m));
Kr = -inv(Cd*inv(Ad-eye(n)-Bd*K)*Bd);

%% Simulation
for t=1:T-1 % t is the timestep number
    %% Plant
    u = U(:,t+1); % grab control inputs u[t]
    x = X(:,t);   % grab state x[t-1]
    [x_new, y] = plant_md(Ad, Bd, Cd, u, x);
    Y(:,t+1) = y;     % save y[t] output in Y matrix
    X(:,t+1) = x_new; % save new state x[t] in X matrix
    
    %% State estimator
    % update observability matrix
    index = (t*p + 1);
    CA(index:index+(p-1),:) = Cd*(Ad^t);
    
    % update matrix of control inputs
    for i=0:t
        Bu(:,t+1) = Bu(:,t+1) + Cd*(Ad^(t-i))*Bd*U(:,i+1);
    end
    
    % run optimization to find initial state x
    YBu = Y(:,1:t+1) - Bu(:,1:t+1);
    CA_t = CA(1:(t+1)*p,:);
    r = 2;
    cvx_begin quiet
        variable x(n)
        minimize( sum(norms(YBu - reshape(CA_t*x,[p,t+1]), r, 2)) )
    cvx_end
    fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)
    
    Xinit(:,t+1) = x;
    
    % propagate dynamics to find previous state from initial state
    for i=1:t
        x_new = Ad*x + Bd*U(:,i+1);
        x = x_new;
    end
    xhat = x_new;
    Xhat(:,t+1) = xhat; % save estmimated state xhat[t] in Xhat matrix
    
    %% Controller
    u_new = Kr*ref - K*xhat; %*X(:,t+1); % calculate new control inputs
    U(:,t+2) = u_new; % save new inputs u[t+1] in U matrix
    
end

%% Plot results
%figure;
hold on
stem(0:Ts:Ttotal-Ts, Xhat(1,:),'DisplayName','estimated state x')
hold on
stem(0:Ts:Ttotal-Ts, Y,'DisplayName','output y/actual state x')
xlabel('time (s)')
ylabel('velocity')
legend('show')
%title('propagate 1 to t-1')


