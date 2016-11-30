%% Inputs
Ttotal = 0.4; % total sample time
Ts = 0.01;    % simulation timestep
p = 1;        % number of sensors
n = 1;        % number of states
m = 1;        % number of inputs

% reference signal (desired state)
ref = zeros(n,1); ref(1) = 1;

% initial state
xhat = zeros(n,1);

% system
c=1; M=1;
A=-c/M; B=1/M; C=1;

%% Initialization and initial calculations
T = Ttotal/Ts; % total timesteps

% initialize matrices to store outputs and inputs
Y = zeros(p,T);
U = zeros(m,T);
X = zeros(n,T);

% initialize matrices for SSE calcs
Bu = zeros(p,T);
CA = zeros(p*T,n);

% determine digital state space equations
sysd = ss(A,B,C,0,Ts);
Ad = sysd.a;
Bd = sysd.b;
Cd = sysd.c;

% control
K = dlqr(Ad,Bd,1,0.1);
Kr = -inv(Cd*inv(Ad-eye(n)-Bd*K)*Bd);

for t=0:T-1 % t is the timestep number
    %% Controller
    U(:,t+1) = Kr*ref - K*xhat;
    
    %% Plant
    u = U(:,t+1); % grab control inputs
    y = plant_md(Ad, Bd, Cd, u, xhat);
    Y(:,t+1) = y; % save outputs in Y matrix
    
    %% State estimator
    % update observability matrix
    index = (t*p + 1);
    CA(index:index+(p-1),:) = C*(A^t);
    
    % update matrix of control inputs
    for i=0:t
        Bu(:,t+1) = Bu(:,t+1) + C*(A^(t-i))*B*U(:,i+1);
    end
    
    % run optimization to find initial state x
    YBu = Y(:,1:t+1) + Bu(:,1:t+1);
    CA_t = CA(1:(t+1)*p,:);
    r = 2;
    cvx_begin quiet
        variable x(n)
        minimize( sum(norms(YBu + reshape(CA_t*x,[p,t+1]), r, 2)) )
    cvx_end
    fprintf('For t=%.2f, cvx problem is %s!\n', t*Ts, cvx_status)
    
    % propagate dynamics to find previous state from initial state
    for i=0:t
        x_new = A*x + B*U(:,i+1);
        x = x_new;
    end
    X(:,t+1) = x_new;
    xhat = x_new;
end

figure;
stem(0:Ts:Ttotal-Ts, Y)
xlabel('time (s)')
ylabel('velocity')


