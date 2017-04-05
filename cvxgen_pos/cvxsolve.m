% Produced by CVXGEN, 2017-03-26 23:36:38 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
CA = params.CA;
YBu = params.YBu;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable x(4, 1);

  minimize(sum(abs(YBu - CA*x)));
cvx_end
vars.x = x;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
