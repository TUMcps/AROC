function res = test_computeTermRegNonlinSysPolyApproach_computeInitialGuessCtrl()
%TEST_COMPUTETERMREGNONLINSYSPOLYAPPROACH_COMPUTEINITIALGUESSCTRL - unit test
%function of checkInvariance.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_checkInvariance
%
% Description:
%       Unit test function for checking constraint satisfaction of the 
%       terminal region. We use an artificial system for which a terminal 
%       region and an invariance-preserving controller can be determined 
%       analytically.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/checkInvariance
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
opts.termRegSysType = termRegSysType.SD;
opts.nonlinSysTermRegAlg = 'poly';
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysPolyApproach(param,opts);

% initialize dynamical system + related settings -------------------------

% artificial system
benchmark = 'test_artificialSystem';

% instantiate dynamics object
dynamics = termRegNonlinSysDynamics(benchmark,opts.termRegSysType, ...
    solver,opts,param);

% compute controller parameters and setup terminalRegion object ----------

% terminal region
set = polyZonotope(opts.xEq,0.5*eye(solver.nx),[], ...
    eye(length(solver.id.termReg)),solver.id.termReg);

% function under test - compute controller parameters
p0 = solver.computeInitialGuessCtrl(dynamics,set);

ctrl = resolve(solver.pZu_vp,p0,solver.id.p);

% assign output argument
termReg = termRegNonlinSysPolyApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,p0,dynamics.dt,solver.H_U,solver.W);

% random simulation ------------------------------------------------------

% -> check feasibility of control inputs and invariance
% function under test
tFinal = opts.timeStep;
resSim = termReg.simulateRandom(tFinal,20,0.5,0.5,2);

% check if the input constraints are satisfied
res_input = checkSimInputs(resSim,param.U);

% check if the end point of the simulation are located inside the terminal
% region
res_inv = checkFinalSimPointsInSet(resSim,termReg.set);

res = res_input && res_inv;

end
