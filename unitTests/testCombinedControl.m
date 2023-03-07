%% Test Combined Control

% load benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 10;                  % number of time steps
Opts.reachSteps = 5;          % number of reachability steps (optimization)
Opts.reachStepsFin = 25;      % num. of reach. steps (final reachable set)
Opts.maxIter = 3;             % max. number of iterations for optimization
Opts.finStateCon = false;     % final reach. set in shifted init. set
Opts.scale = [0.3;0.7];       % scaling factor for tightend input set
Opts.Q = diag([0.5 1 1 1]);   % state weighting matrix
Opts.R = 0.02*eye(2);         % input weighting matrix
Opts.Qff = diag([1,10,1,1]);  % state weighting matrix (feedforward contr.)
Opts.Rff = 0.01*eye(2);       % input weighting matrix (feedforward contr.)
Opts.refTraj.Q = eye(4);      % state weighting matrix (reference traj.)
Opts.refTraj.R = Opts.Rff;    % input weighting matrix (reference traj.)

% offline-phase computations
[obj,result] = combinedControl('car',Param,Opts);

% simulation 
[tmp,~,~] = simulateRandom(obj);
result = add(result,tmp);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation are located inside the final 
% reachable set
res = checkFinalSimPoints(result);
assert(res == 1);

% check if the final reachable set is contained in the shifted initial set
res = checkFinalInInitSet(result,Param.R0);
assert(res == 1);