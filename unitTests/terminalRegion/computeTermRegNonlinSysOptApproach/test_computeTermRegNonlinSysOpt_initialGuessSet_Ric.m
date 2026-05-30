function res = test_computeTermRegNonlinSysOpt_initialGuessSet_Ric
%TEST_COMPUTETERMREGNONLINSYSOPT_INITIALGUESSSET_RIC - unit test
%function of initialGuessSet_RiccatiEq.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_initialGuessSet_Ric
%
% Description:
%        Checks whether the computed initial guess for the terminal region
%        is translated to the equilbrium state, is non-degenerate and has
%        the desired zonotope order.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/initialGuessSet_RiccatiEq
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

% longitudinal quadrotor 
benchmark = 'longQuadrotor';

% benchmark parameters
% ... set of admissible states
lb = [-1.7; 0.3; -0.8; -1; -pi/12; -pi/2]; 
ub = [1.7; 2.0; 0.8; 1.0; pi/12; pi/2];
Param.X = interval(lb, ub);
Opts.nx = Param.X.dim;
% set of admissible control inputs
g = 9.81; K = 0.89/1.4;
lb = [-1.5+g/K; -pi/12]; ub = [1.5+g/K; pi/12];
Param.U = interval(lb, ub);
Opts.nu = Param.U.dim;
% set of uncertain disturbances
width = [0.1; 0.01]; 
Param.W = interval(-width, width);
Opts.nw = Param.W.dim;

% ... algorithm settings
Opts.timeStep = 0.05;
g = 9.81; K = 0.89/1.4;
Opts.xEq = [0;1;0;0;0;0];
Opts.uEq = [g/K;0];
Opts.termRegOrder = 5;
Opts.Q = eye(6);
Opts.R = 0.1*eye(2);
Opts.nonlinSysTermRegAlg = 'lin';
Opts.sysType = termRegSysType.DT;
Opts.integrator = discretizeDynamics.RK4;
Opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(Param,Opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,Opts.sysType,solver,Opts,Param);

% test and assign output argument
termReg = initialGuessSet_RiccatiEq(solver,dynamics,Opts.Q,Opts.R);
if all(withinTol(termReg.set.c,Opts.xEq)) ...
        && size(termReg.set.G,2) == Opts.nx*Opts.termRegOrder ...
        && termReg.set.isFullDim()
    res = true;
else
    res = false;
end

end
