
warning('off', 'MATLAB:nargchk:deprecated');

%% Define constants
p = 4; % p is the number of sensors
%        How to handle different number of sensors and sensor outputs?
n = 8; % n is the number of states

%% System parameters
A = randn(n);
C = randn(p,n); % property of the system (linearized dynamic model?)

%% Sensor outputs
y0 = zeros(p,1);
y1 = zeros(p,1);
y2 = zeros(p,1);
Y = [y0 y1 y2]; % Y has as size sensors outputs p by timesteps T

%% Actual algorithm
cvx_begin
	variable x(n) % x is a variable with length n (number of states)
	minimize( sum(norms(Y - [C*x C*A*x C*(A^2)*x], Inf, 2)) ) % right kind of norm?
	subject to
		isreal(x)
cvx_end

% http://web.cvxr.com/cvx/doc/funcref.html#funcref <-- see here for "norms" documentation
