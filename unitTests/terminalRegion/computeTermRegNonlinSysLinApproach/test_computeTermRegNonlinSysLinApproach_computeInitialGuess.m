function res = test_computeTermRegNonlinSysLinApproach_computeInitialGuess
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_INITIALGUESS - unit test function
%of computeInitialGuess.
%
% Syntax: 
%       res = test_computeTermRegNonlinSysLinApproach_computeInitialGuess
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/initialGuess
%
% References:
%       -
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
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% dynamical system
benchmark = 'cart';

% sets of admissible states, control inputs, and disturbances
width = [6; 6];
param.X = interval(-width, width);
width = 14;
param.U = zonotope(interval(-width,width));
width = [0.1;0.1];
W = interval(-width,width);
param.W = zonotope(W);

% ... algorithm settings
opts.nonlinSysTermRegAlg = 'lin';
opts.timeStep = 0.05;
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.termRegOrder = 5;
opts.verbose = 0;

% optional algorithm settings
opts_initialGuess.Q = eye(param.X.dim);
opts_initialGuess.R = 0.1*eye(param.U.dim);
opts_initialGuess.weights.RiccatiEq = 1.0;
opts_initialGuess.weights.minRCI = 1.0; 
opts_initialGuess.weights.minRPI = 1.0;
opts_initialGuess.weights.bwReachInputs = 1.0;

% ---------------- test: discrete-time system ----------------------------
opts.sysType = termRegSysType.DT;
opts.integrator = discretizeDynamics.RK2;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% initialize dynamics
dynamics = termRegNonlinSysDynamics( ...
    benchmark,opts.sysType,solver,opts,param);

% function under test
initGuess = computeInitialGuess( ...
    solver,dynamics,opts_initialGuess.Q,opts_initialGuess.R,opts_initialGuess.weights);

res_DT = all(withinTol(initGuess.set.c,opts.xEq,eps)) ...
    && size(initGuess.set.G,2) == opts.termRegOrder*param.X.dim;

opts = rmfield(opts,'sysType');
opts = rmfield(opts,'integrator');

% ---------------- test: sampled-data system -----------------------------
opts.sysType = termRegSysType.SD;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% initialize dynamics
dynamics = termRegNonlinSysDynamics( ...
    benchmark,opts.sysType,solver,opts,param);

% function under test
initGuess = computeInitialGuess(solver,dynamics, ...
    opts_initialGuess.Q,opts_initialGuess.R,opts_initialGuess.weights);

res_SD = all(withinTol(initGuess.set.c,opts.xEq,eps)) ...
    && size(initGuess.set.G,2) == opts.termRegOrder*param.X.dim;

res = res_DT && res_SD;

end
