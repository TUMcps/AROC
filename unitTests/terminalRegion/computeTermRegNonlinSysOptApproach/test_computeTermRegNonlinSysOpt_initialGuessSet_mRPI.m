function res = test_computeTermRegNonlinSysOpt_initialGuessSet_mRPI
%TEST_COMPUTETERMREGNONLINSYSOPT_INITIALGUESSSET_MRPI - unit test 
% function of initialGuessSet_mRPI.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_initialGuessSet_mRPI
%
% Description:
%       To test the computation of an invariant outer approximation of the
%       minimal robust positively invariant (mRPI) set, we sample initial 
%       states from the computed set, simulate the system, and check 
%       containment of the simulated trajectory. 
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/initialGuessSet_mRPI
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

nRuns = 100;
nSteps = 25;

% cart system ------------------------------------------------------------

benchmark = 'cart';

% set parameters for computing the mRPI set
% ... set of admissible states
width = [6; 6];
param.X = interval(-width, width);
opts.nx = param.X.dim;
% ... set of admissible control inputs 
width = 14;
param.U = interval(-width,width);
opts.nu = param.U.dim;
% ... set of uncertain disturbances 
width = [0.1;0.1];
param.W = zonotope(interval(-width,width));
opts.nw = param.W.dim;

% ... algorithm settings (no weighting matrices)
opts.timeStep = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = 0;
opts.termRegOrder = 5;
opts.Q = eye(2);
opts.R = 0.1*eye(1);
opts.nonlinSysTermRegAlg = 'lin';
opts.sysType = termRegSysType.DT;
opts.integrator = discretizeDynamics.RK4;
opts.verbose = 0;

% test - invariance and centering at equilibrium state
[res_cart,termReg] = runAndSim_test_terminalRegion_initialGuess_mRPI( ...
    benchmark,opts,param,nRuns,nSteps);
if ~all(withinTol(termReg.c,opts.xEq))
    res_cart = false;
end

clear opts param

% longitudinal quadrotor -------------------------------------------------
% only test translation of mRPI set (to opts.xEq) and test omitting
% optional input arguments

benchmark = 'longQuadrotor';

% set parameters for computing the mRPI set
% ... set of admissible states
lb = [-1.7; 0.3; -0.8; -1; -pi/12; -pi/2]; 
ub = [1.7; 2.0; 0.8; 1.0; pi/12; pi/2];
param.X = interval(lb, ub);
opts.nx = param.X.dim;
% set of admissible control inputs
g = 9.81; K = 0.89/1.4;
lb = [-1.5+g/K; -pi/12]; ub = [1.5+g/K; pi/12];
param.U = interval(lb, ub);
opts.nu = param.U.dim;
% set of uncertain disturbances
width = [0.1; 0.01]; 
param.W = interval(-width, width);
opts.nw = param.W.dim;

% ... algorithm settings
opts.timeStep = 0.05;
g = 9.81; K = 0.89/1.4;
opts.xEq = [0;1;0;0;0;0];
opts.uEq = [g/K;0];
opts.termRegOrder = 5;
opts.Q = eye(6);
opts.R = 0.1*eye(2);
opts.nonlinSysTermRegAlg = 'lin';
opts.sysType = termRegSysType.DT;
opts.integrator = discretizeDynamics.RK4;
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% test
res_quadrotor = false(2,1);
% -> no optional input arguments
termReg = initialGuessSet_mRPI(solver,dynamics,opts.Q,opts.R);
if all(withinTol(termReg.set.c,opts.xEq)) ...
        && size(termReg.set.G,2) == opts.nx*opts.termRegOrder
    res_quadrotor(1) = true;
end
% -> no invariance of termional region required
termReg = initialGuessSet_mRPI(solver,dynamics,opts.Q,opts.R,false);
if all(withinTol(termReg.set.c,opts.xEq)) ...
        && size(termReg.set.G,2) == opts.nx*opts.termRegOrder
    res_quadrotor(2) = true;
end
% -> requiring invariance of the terminal region required is considered in
% the cart tests

% assign output argument
res = res_cart && all(res_quadrotor);

end


function [res,termReg] = runAndSim_test_terminalRegion_initialGuess_mRPI( ...
    benchmark,opts,param,nRuns,nSteps)
    
    % instantiate solver object
    solver = computeTermRegNonlinSysLinApproach(param,opts);
    
    % instantiate dynamics
    dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver, ...
        opts,param);

    % compute mRPI set and stabilizing controller
    termReg = initialGuessSet_mRPI(solver,dynamics,opts.Q,opts.R,true);


    % ------------ random simulation to check invariance -----------------

    % function handle to dynamics function
    str = ['dynamicsHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % dummy reachability settings for computing derivatives
    reachOpts.tensorOrder = 2;
    reachOpts.zonotopeOrder = 100;
    reachOpts.errorOrder = 5;
    reachOpts.reductionTechnique = 'pca';
    
    % discretize dynamical system
    integrator_fH = opts.integrator.getFunHandle;
    dynamicsHandleDT = @(x,u,w) integrator_fH(x,u,w,dynamicsHandle,opts.timeStep);            
    nlSysDT = nonlinearSysDT(['test_terminalRegion_',benchmark], ...
        @(x,w) [dynamicsHandleDT(x(1:opts.nx),x(opts.nx+1:opts.nx+opts.nu),w); ...
        zeros(opts.nu,1)], ...
        opts.timeStep,opts.nx+opts.nu, opts.nw);
    derivatives(nlSysDT,reachOpts); 
    
    % linearize dynamics and compute closed-loop system
    [jac_xu,jac_w] = nlSysDT.jacobian([opts.xEq;opts.uEq],zeros(opts.nw,1));
    Ad = jac_xu(1:opts.nx,1:opts.nx);
    Bd = jac_xu(1:opts.nx,opts.nx+(1:opts.nu));
    Ed = jac_w(1:opts.nx,:);
    
    % closed-loop simulation
    res = true(nRuns,1);
    X0 = randPoint(termReg.set,nRuns,'uniform');
    for ii = 1:nRuns
        % sample disturbance trajectory
        W = randPoint(param.W,nSteps,'extreme');
        % simulation
        x = X0(:,ii);
        for kk = 1:nSteps
            % compute control input
            u = termReg.computeControlInput(x);
            % simulate
            x = Ad*(x-opts.xEq) + Bd*(u-opts.uEq) + Ed*W(:,kk) + opts.xEq;
            if ~contains(termReg.set,x)
                res(ii) = false;
            end
        end
    end

res = all(res);

termReg = termReg.set;

end
