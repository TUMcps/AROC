function res = test_computeTermRegNonlinSysOpt_checkInvariance
%TEST_COMPUTETERMREGNONLINSYSOPT_CHECKINVARIANCE - unit test
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
opts.integrator = discretizeDynamics.RK2;
opts.nonlinSysTermRegAlg = 'lin';
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% initialize dynamical system + related settings -------------------------

% artificial system
benchmark = 'test_artificialSystem';

% instantiate dynamics object
dynamics = termRegNonlinSysDynamics(benchmark,termRegSysType.DT, ...
    solver,opts,param);

% feasible solution to be verified ---------------------------------------

% ... terminal region
set = zonotope(opts.xEq,0.5*eye(solver.nx));
% ... invariance-preserving controller 
K = -eye(solver.nx);
ctrl = zonotope(opts.uEq,K*set.G);
termReg_feas = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,dynamics.dt,solver.Z_U,solver.W);

% execute function under test
[flagInvariance,flagStateConstraints,b_tight] = solver.checkInvariance(dynamics,termReg_feas);
resFeas = flagInvariance && flagStateConstraints && all(b_tight==0);

% infeasible solution to be verified -------------------------------------

% ... terminal region
set = zonotope(opts.xEq,0.5*eye(solver.nx));
% ... invariance-preserving controller 
K = -0.1*eye(solver.nx);
ctrl = zonotope(opts.uEq,K*set.G);
termReg_infeas = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,dynamics.dt,solver.Z_U,solver.W);

% execute function under test
[flagInvariance,flagStateConstraints,b_tight] = solver.checkInvariance(dynamics,termReg_infeas);
resInfeas = ~flagInvariance && flagStateConstraints && all(b_tight==0);

% terminal region not contained in X (discrete-time system) --------------

% ... terminal region = set of admissible states
set = zonotope(opts.xEq,diag([1.001,0.9]));
% ... invariance-preserving controller 
K = -eye(solver.nx);
ctrl = zonotope(opts.uEq,K*set.G);
termReg_Xinfeas = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,dynamics.dt,solver.Z_U,solver.W);

% execute function under test
[flagInvariance,flagStateConstraints,b_tight] = solver.checkInvariance(dynamics,termReg_Xinfeas);
resXinfeas = flagInvariance && ~flagStateConstraints && any(b_tight > 0);

% time-interval constraint violated (sampled-data system) ----------------

% instantiate dynamics object
dynamics = termRegNonlinSysDynamics(benchmark,termRegSysType.SD,solver,opts,param);

% ... terminal region = set of admissible states
set = zonotope(opts.xEq,eye(solver.nx));
% ... invariance-preserving controller 
K = -eye(solver.nx);
ctrl = zonotope(opts.uEq,K*set.G);
termReg_tiInfeas = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,dynamics.dt,solver.Z_U,solver.W);

% execute function under test
[flagInvariance,flagStateConstraints,b_tight] = solver.checkInvariance(dynamics,termReg_tiInfeas);
resTiInfeas = flagInvariance && ~flagStateConstraints && any(b_tight > 0);

% poly-approach ----------------------------------------------------------

opts.nonlinSysTermRegAlg = 'poly';

% solver object
solver = computeTermRegNonlinSysPolyApproach(param,opts);

% instantiate dynamics object (sampled-data system
dynamics = termRegNonlinSysDynamics(benchmark,termRegSysType.SD,solver,opts,param);

% case feasible solution
% ... terminal region
set = polyZonotope(opts.xEq,0.5*eye(solver.nx),[], ...
    eye(length(solver.id.termReg)),solver.id.termReg);
% ... invariance-preserving controller 
K = -eye(solver.nx);
ctrl = polyZonotope(opts.uEq,K*set.G,[], ...
    eye(length(solver.id.termReg)),solver.id.termReg);
termReg_feas = termRegNonlinSysPolyApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set,ctrl,rand(solver.nu*(1+solver.termRegOrder),1), ...
    dynamics.dt,solver.Z_U,solver.W);

% execute function under test
[flagInvariance,flagStateConstraints,b_tight] = solver.checkInvariance(dynamics,termReg_feas);
resPoly = flagInvariance && flagStateConstraints && all(b_tight==0);


% collect results --------------------------------------------------------

res = resFeas && resInfeas && resXinfeas && resTiInfeas && resPoly;

end
