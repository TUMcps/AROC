function res = test_termRegNonlinSysOpt_simulateRandom()
%TEST_termRegNonlinSysOpt_SIMULATERANDOM - unit test function for
%simulateRandom.
%
% Syntax:
%       res = test_termRegNonlinSysOpt_simulateRandom
%
% Description:
%       Given a terminal region and its associated terminal controller, we 
%       check whether the computed control inputs satisfy the input
%       constraints and whether the simulated trajectories stay in the
%       temrinal region (at the end of the simulation horizon). If the
%       terminal region is left at any sampling instant, the computed
%       control inputs would violate the input constraints. Thus, we only
%       have to check invariance for the end point of the trajectory.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/simulateRandom
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

% sampled-data system ----------------------------------------------------

% artificial system
benchmark = 'test_artificialSystem';

% set of admissible states
param.X = interval(-ones(2,1), ones(2,1));
% set of admissible control inputs
param.U = interval(-ones(2,1), ones(2,1));
% set of uncertain disturbances
param.W = interval(-0.1*ones(2,1), 0.1*ones(2,1));

% algorithm settings
opts.timeStep = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = zeros(2,1);
opts.termRegOrder = 1;
opts.nonlinSysTermRegAlg = 'lin';
opts.sysType = termRegSysType.SD;
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics object
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType, ...
    solver,opts,param);

% terminal region object
set = zonotope(opts.xEq,0.5*eye(solver.nx));
K = -eye(solver.nx);
ctrl = zonotope(opts.uEq,K*set.G);
termReg = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,dynamics.dt,solver.Z_U,solver.W);

% function under test
tFinal = 10.0;
resSim = termReg.simulateRandom(tFinal,5,0.5,0.5,2);

% check if the input constraints are satisfied
res_input = checkSimInputs(resSim,param.U);

% check if the end point of the simulation are located inside the terminal
% region
res_inv = checkFinalSimPointsInSet(resSim,termReg.set);

resSD = res_input && res_inv;

% discrete-time system ---------------------------------------------------

% dynamics function 
dynamicsHandle = @(x,u,w) [0, 1; -1, 0]*x + [1, 1; -1, 1]*u + ones(2,1)*w;

% set of admissible states
param.X = interval(-ones(2,1), ones(2,1));
% set of admissible control inputs
param.U = 0.2*interval(-ones(2,1), ones(2,1));
% set of uncertain disturbances
param.W = 0.2*interval(-1, 1);

% algorithm settings
opts.timeStep = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = zeros(2,1);
opts.termRegOrder = 1;
opts.nonlinSysTermRegAlg = 'lin';
opts.sysType = termRegSysType.DT;
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% terminal region object
set = zonotope(opts.xEq,eye(solver.nx));
ctrl = zonotope(opts.uEq, [-0.1,-0.1; 0.1,-0.1]);
termReg = termRegNonlinSysLinApproach(dynamicsHandle, ...
    opts.sysType,set,ctrl,opts.timeStep,solver.Z_U,solver.W);

% function under test
tFinal = 10.0;
resSim = termReg.simulateRandom(tFinal,5,0.5,0.5,2);

% check if the input constraints are satisfied
res_input = checkSimInputs(resSim,param.U);

% check if the end point of the simulation are located inside the terminal
% region
res_inv = checkFinalSimPointsInSet(resSim,termReg.set);

resDT = res_input && res_inv;

% assign output argument -------------------------------------------------

res = resSD && resDT;

end
