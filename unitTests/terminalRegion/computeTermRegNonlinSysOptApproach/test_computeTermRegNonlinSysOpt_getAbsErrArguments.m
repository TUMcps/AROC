function res = test_computeTermRegNonlinSysOpt_getAbsErrArguments()
%TEST_COMPUTETERMREGNONLINSYSOPT_GETABSERRARGUMENTS - unit test
%function for getAbsErrArguments.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_computeAbsErr
%
% Description:
%       Unit test function for getAbsErrArguments.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/getAbsErrArguments
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

% ----------- linearization-based approach -------------------------------
% -> jet engine (only one state enters the abstraction error)
benchmark = 'controlledJetEngine';

% set of admissible states, control inputs, and disturbances
width = [0.2; 0.2];  
param.X = interval(-width, width);
param.U = interval(-0.35, 0.35);
param.W = interval(-0.025, 0.025);

% algorithm settings
opts.nonlinSysTermRegAlg = 'lin';
opts.sysType = termRegSysType.SD;
opts.timeStep = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = 0;
opts.termRegOrder = 5;
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% function under test
solver = solver.getAbsErrArguments(dynamics);

% check results
if isequal(solver.argAbsErr,1)
    res_linJet = true;
else
    res_linJet = false;
end

% ----------- linearization-based approach -------------------------------
% -> cart (both states enter the abstraction error) 
% dynamical system
benchmark = 'cart';

% sets of admissible states, control inputs, and disturbances
width = [6; 6];
param.X = interval(-width, width);
width = 14;
param.U = zonotope(interval(-width,width));
width = [0.1;0.1];
param.W = interval(-width,width);

% algorithm settings
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.timeStep = 0.1;
opts.termRegOrder = 5;
opts.sysType = termRegSysType.SD;
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% function under test
solver = solver.getAbsErrArguments(dynamics);

% check results
if isequal(solver.argAbsErr,[1;2])
    res_linCart = true;
else
    res_linCart = false;
end

% ----------- polyomialization-based approach ----------------------------

opts.nonlinSysTermRegAlg = 'poly';

% instantiate solver object
solver = computeTermRegNonlinSysPolyApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% function under test
solver = solver.getAbsErrArguments(dynamics);

% check results
if isequal(solver.argAbsErr,1)
    res_poly = true;
else
    res_poly = false;
end

% assign output argument
res = res_linJet && res_linCart && res_poly;

end
