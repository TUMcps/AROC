%% Test Reachset Model Predictive Control

% load benchmark parameter
Param = param_stirredTankReactor();
Param.x0 = [-0.3;-30];

% define algorithm options
Opts = [];

Opts.N = 5;                     % number of time steps
Opts.reachSteps = 1;            % number of reachability steps
Opts.tOpt = 9;                  % final time for optimization horizon
Opts.Q = diag([100,1]);         % state weighting matrix  
Opts.R = 0.9;                   % input weighting matrix
Opts.Qlqr = diag([1;1]);        % state weighting matrix (feedback control)
Opts.Rlqr = 100;                % input weighting matrix (feedback control)
Opts.scale = 0.9556;            % scaling factor for tightend input set
Opts.tComp = 0.54;              % allocated computation time
Opts.alpha = 0.1;               % contraction rate
Opts.maxIter = 50;              % max number of iterations for opt. control

% terminal region (see Sec. IV in Schuermann et al. (2018): "Reachset Model 
% Predictive Control for Disturbed Nonlinear Systems", CDC)
A = [-1.0000 0;1.0000 0;30.0000 -1.0000;66.6526 -4.8603;-66.6526 4.8603];
b = [0.3000;0.0620;11.8400;65.0000;15.0000];

Opts.termReg = mptPolytope(A,b);

% offline phase computations
result = reachsetMPC('stirredTankReactor',Param,Opts);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation is inside the terminal region
p = getFinalSimPoints(result);
res = contains(Opts.termReg,p);
assert(res == 1);

% check if the final reachable set is located inside the terminal region
res = contains(Opts.termReg,result.reachSetTimePoint{end});
assert(res == 1);