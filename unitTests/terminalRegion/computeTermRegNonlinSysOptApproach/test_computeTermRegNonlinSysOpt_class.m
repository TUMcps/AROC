function res = test_computeTermRegNonlinSysOpt_class
%TEST_COMPUTETERMREGNONLINSYSOPT_CLASS - unit test function of
%methods defined in the class file computeTermRegNonlinSysOpt.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_class
%
% Description:
%       Unit test function for methods defined in the class file 
%       computeTermRegNonlinSysOpt.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt
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

% instantiate solver object
% sets of admissible states, control inputs, and disturbances
width = [6; 6];
param.X = interval(-width, width);
nx = param.X.dim;
width = 14;
param.U = zonotope(interval(-width,width));
width = [0.1;0.1];
param.W = interval(-width,width);

% algorithm settings
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.termRegOrder = 5;
opts.verbose = 0;

solver = computeTermRegNonlinSysOpt(param.X,param.U,param.W, ...
    opts.xEq,opts.uEq,opts.termRegOrder,opts.verbose);

nGen = nx*opts.termRegOrder;

% --------------- tighten constraint set ---------------------------------
res_tighthen = false(2,1);

% test case 1: tighten constraint set
b_tight_pos = rand(4,1);
solver_tc1 = solver.tightenStateConstraintSet(b_tight_pos);

% ... check results: H_X_con contained in H_X and H_X identical to X
res_tighthen(1) = contains(solver_tc1.H_X,solver_tc1.H_X_con) && ...
    (contains(param.X,solver_tc1.H_X) && contains(solver_tc1.H_X,param.X));

% test case 2: no bloating of constraint set possible
b_tight_neg = rand(4,1) - 1;
try
    % execution expected to fail due to input argument check
    solver.tightenStateConstraintSet(b_tight_neg);
    res_tighthen(2) = false;
catch
    res_tighthen(2) = true;
end

% --------------- update trust region ------------------------------------
res_tr = false(3,1);

% test case 1: interval centered at origin (represented as a zonotope)
tr_new = zonotope(zeros(nx,1),diag(rand(nx,1)));
try
    solver.tr_set = tr_new;
    res_tr(1) = contains(solver.tr_set,tr_new) && contains(tr_new,solver.tr_set);
catch
    res_tr(1) = false;
end

% test case 2: trust region not centered at the origin
tr_new = zonotope(rand(nx,1),diag(rand(nx,1)));
try
    solver.tr_set = tr_new;
    res_tr(2) = false;
catch
    res_tr(2) = true;
end

% test case 3: trust region is not an interval
tr_new = zonotope.generateRandom('Center',zeros(nx,1));
try
    solver.tr_set = tr_new;
    res_tr(3) = false;
catch
    res_tr(3) = true;
end

% -------------- volume of cost set --------------------------------------
res_cost = false(2,1);

% test case 1: no reduction reduired
opts.solverSettings.maxCostTerms ...
    = factorial(nGen)/(factorial(nx)*factorial(nGen-nx));
solver_tc1 = computeTermRegNonlinSysOpt(param.X,param.U,param.W, ...
    opts.xEq,opts.uEq,opts.termRegOrder,opts.verbose,opts.solverSettings);

% ... generate random initial guess
initialGuess = zonotope.generateRandom('Center',solver_tc1.xEq,'NrGenerators',nGen);

% ... compute cost function coefficients and check result
solver_tc1 = solver_tc1.initCostFunction(initialGuess);
res_cost(1) = withinTol(solver_tc1.costSetVolume(),volume(initialGuess));

% test case 2: reduction required
opts.solverSettings.maxCostTerms ...
    = floor(0.5*factorial(nGen)/(factorial(nx)*factorial(nGen-nx)));
solver_tc2 = computeTermRegNonlinSysOpt(param.X,param.U,param.W, ...
    opts.xEq,opts.uEq,opts.termRegOrder,opts.verbose,opts.solverSettings);

% ... generate random initial guess
initialGuess = zonotope.generateRandom('Center',solver_tc2.xEq,'NrGenerators',nGen);

% ... compute cost function coefficients and check result
solver_tc2 = solver_tc2.initCostFunction(initialGuess);
res_cost(2) = (volume(initialGuess) > solver_tc2.costSetVolume()) ...
    && withinTol(solver_tc2.costSetVolume(),volume(solver_tc2.costSet));

% assign output argument
res = all(res_tighthen) && all(res_tr) && all(res_cost);

end
