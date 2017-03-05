% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(norm(Ip_1*(YBu - CA*x), 1) + norm(Ip_2*(YBu - CA*x), 1) + norm(Ip_3*(YBu - CA*x), 1) + norm(Ip_4*(YBu - CA*x), 1) + norm(Ip_5*(YBu - CA*x), 1))
%
% with variables
%        x   5 x 1
%
% and parameters
%       CA  15 x 5
%     Ip_1   3 x 15
%     Ip_2   3 x 15
%     Ip_3   3 x 15
%     Ip_4   3 x 15
%     Ip_5   3 x 15
%      YBu  15 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.CA, ..., params.YBu, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2017-02-21 01:47:42 -0500.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
