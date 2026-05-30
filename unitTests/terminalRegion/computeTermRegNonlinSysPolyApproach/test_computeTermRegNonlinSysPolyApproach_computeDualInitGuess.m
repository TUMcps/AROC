function res = test_computeTermRegNonlinSysPolyApproach_computeDualInitGuess()
%TEST_COMPUTETERMREGNONLINSYSPOLYAPPROACH_COMPUTEDUALINITGUESS - unit test
%function of computeDualInitialGuess.
%
% Syntax:
%       res = test_computeTermRegNonlinSysPolyApproach_computeDualInitGuess()
% 
% Description:
%       Simple 2-dimensional test cases. All test cases are chosen so that
%       LICQ holds at the stationary point so that the values of the
%       Lagrange multipliers are unique.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant 
%           Sets of Nonlinear Systems Using Polynomial Controller 
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Synthesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------        
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% cost function
cost = @(x) x(1)^2 + x(2)^2;

% variable structure
idxVars.nVars = 2;
idxVars.slack = [];
idxVars.n_slack = 0;

% candidate point
y0 = [0; 1];

% test case: two active inequality constraint ----------------------------

% inquality constraints: -x(1)+x(2) >= 1, x(1)+x(2) >= 1
con = @(x) [x(1) - x(2); -x(1) - x(2)];
cu = [-1; -1];
cl = -inf(2,1);

% constraint structure
idxCon.nCon = 2;
idxCon.ineqCon = [1 ,2];

% variable bounds
lbx = -inf;
ubx = inf;

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = [1; 1];
lam_x_exp = [0; 0];

% check results
res_case1 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% test case: lower variable bound active and inequality constraint  ------

% inequality constraints: -x(1)+x(2) >= 1
con = @(x) x(1) - x(2);
cu = -1;
cl = -inf;

% constraint structure
idxCon.nCon = 1;
idxCon.ineqCon = 1;

% variable bounds: x(1) >= 0
lbx = [0; -inf];
ubx = [inf; inf];

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = 2;
lam_x_exp = [2;0];

% check results
res_case2 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% test case: upper variable bound active and inequality constraint  ------

% inequality constraints: x(1)+x(2) >= 1
con = @(x) -x(1) - x(2);
cu = -1;
cl = -inf;

% constraint structure
idxCon.nCon = 1;
idxCon.ineqCon = 1;

% variable bounds: x(1) <= 0
lbx = [-inf;-inf];
ubx = [0;inf];

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = 2;
lam_x_exp = [2;0];

% check results
res_case3 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% test case equality and inequality constraint ---------------------------

% constraints: -x(1)+x(2) >= 1, -x(1) - x(2)=-1
con = @(x) [x(1) - x(2); x(1) + x(2)];
cu = [-1;1];
cl = [-inf;1];


% constraint structure
idxCon.nCon = 2;
idxCon.ineqCon = 1;
idxCon.eqCon = 2;

% variable bounds
lbx = -inf;
ubx = inf;

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = [1; -1];
lam_x_exp = [0; 0];

% check results
res_case4 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% test case inequality constraint and equality constrained variable bound 

% constraint: -x(1)+x(2) >= 1
con = @(x) x(1) - x(2);
cu = -1;
cl = -inf;

% constraint structure
idxCon.nCon = 1;
idxCon.ineqCon = 1;

% variable bounds: x(1) = 0
lbx = [0; -inf];
ubx = [0; inf];

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = 2;
lam_x_exp = [-2; 0];

% check results
res_case5 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% test case: no active constraint ----------------------------------------

% candidate point
y0 = zeros(2,1);

% inquality constraints: -x(1)+x(2) >= 1, x(1)+x(2) >= 1
con = @(x) [x(1) - x(2); -x(1) - x(2)];
cu = [-1; -1];
cl = -inf(2,1);

% constraint structure
idxCon.nCon = 2;
idxCon.ineqCon = [1 ,2];

% variable bounds
lbx = -inf;
ubx = inf;

% function under test
[lam_g,lam_x] = computeTermRegNonlinSysPolyApproach.computeDualInitialGuess(y0,cost,con,idxVars,idxCon,lbx,ubx);

% expected result
lam_g_exp = [0; 0];
lam_x_exp = [0; 0];

% check results
res_case6 = norm(lam_g-lam_g_exp,'inf') <= 1e-6 && norm(lam_x-lam_x_exp,'inf') <= 1e-6;

% collect results --------------------------------------------------------
 
res = res_case1 && res_case2 && res_case3 && res_case4 && res_case5 && res_case6;

end
