%% Test LQR Comfort Controller

% load benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.reachSteps = 5;            % number of reachability steps
Opts.tComp = 0.05;              % allocated computation time 
Opts.refTraj.Q = 10*eye(4);     % state weighting matrix (reference traj.)        
Opts.refTraj.R = 1/10*eye(2);   % input weighting matrix (reference traj.)
    
% settings for the comfort controller that is applied online
Opts.controller = {'LQR','LQR'};

Q = diag([0.2 10 31.2 1]);
contrOpts1.R = diag([50,170]); contrOpts1.Q = Q;
contrOpts2.R = diag([60,200]); contrOpts2.Q = Q;

Opts.contrOpts = {contrOpts1,contrOpts2};

% offline phase computations
[obj,result] = safetyNetControl('car',Param,Opts);

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
res = checkFinalInInitSet(result,result.reachSetTimePoint{1});
assert(res == 1);


%% Test MPC Comfort Controller

% load benchmark parameter
Param = param_car();

% settings for the comfort controller that is applied online
Opts.controller = 'MPC';
contrOpts.Q = diag([12 10 60 35]);
contrOpts.R = diag([0.1 1]);
contrOpts.horizon = 3;
contrOpts.Ninter = 2;
Opts.contrOpts = contrOpts;

% offline phase computations
[obj,result] = safetyNetControl('car',Param,Opts);

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
res = checkFinalInInitSet(result,result.reachSetTimePoint{1});
assert(res == 1);