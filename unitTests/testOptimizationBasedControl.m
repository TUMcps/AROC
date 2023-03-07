%% Test Linear System

% get benchmark parameter
Param = param_platoon();

% define algorithm options
Opts = [];

Opts.reachSteps = 11;           % number of reachability steps
Opts.maxIter = 200;             % max number of iterations for optimization
Opts.bound = 10000;             % diff. bet. upper and lower bound of vars.
Opts.refTraj.Q = eye(8);        % state weighting matrix (reference traj.)
Opts.refTraj.R = 0.02*eye(4);   % input weighting matrix (reference traj.)

% offline phase computations
[objContr,result] = optimizationBasedControl('platoon',Param,Opts);

% simulation 
[tmp,~,~] = simulateRandom(objContr);
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


%% Test Nonlinear System

% get benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 10;                    % number of time steps  
Opts.reachSteps = 12;           % number of reachability steps
Opts.maxIter = 10;              % max number of iterations for optimization
Opts.bound = 10000;             % diff. bet. upper and lower bound of vars.
Opts.refTraj.Q = 10*eye(4);     % state weighting matrix (reference traj.)
Opts.refTraj.R = 1/10*eye(2);   % input weighting matrix (reference traj.)

% offline phase computations
[objContr,result] = optimizationBasedControl('car',Param,Opts);

% simulation 
[tmp,~,~] = simulateRandom(objContr);
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