
% suppress warnings to reduce the amount of text printed
warning('off', 'MATLAB:nargchk:deprecated');

%% Define constants
% These numbers are all chosen arbitrarily for now
p = 4; % p is the number of sensor outputs
n = 6; % n is the number of states
r = 2; % r is the l_r norm used by the decoder

%% System parameters
% Arbitrary random matricies because we don't have the system matrices
%     from the dynamic model yet
A = randn(n);
C = randn(p,n); % A and C are properties of the system

%% Sensor outputs
y0 = zeros(p,1);
y1 = zeros(p,1);
y2 = zeros(p,1);
Y = [y0 y1 y2]; % Y has a size sensor outputs p by timesteps T

%% Actual algorithm
cvx_begin
	variable x(n) % x is a variable with length n (number of states)
	minimize( sum(norms(Y - [C*x C*A*x C*(A^2)*x], r, 2)) )
	subject to
		isreal(x)
cvx_end

% http://web.cvxr.com/cvx/doc/funcref.html#funcref
% ^ documentation for norms function in CVX
