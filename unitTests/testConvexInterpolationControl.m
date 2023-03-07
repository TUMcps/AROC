%% Test Linear Control Law

% get benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.Q = diag([2,5,1,1]);             % state weighting matrix
Opts.R = diag([0;0]);                 % input weighting matrix
Opts.refTraj.Q = 10*eye(4);           % state weighting matrix (ref. traj.)
Opts.refTraj.R = 1/10*eye(2);         % input weighting matrix (ref. traj.)
Opts.controller = 'linear';           % controller

% offline phase computations
[obj,result] = convexInterpolationControl('car',Param,Opts);

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


%% Test Exact Control Law

% get benchmark parameter
Param = param_car();

% adapt algorithm options
Opts.controller = 'exact';
Opts.polyZono.N = 5;

% offline phase computations
[obj,result] = convexInterpolationControl('car',Param,Opts);

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


%% Test Extendet Optimization Horizon

% load benchmark parameter
Param = param_cart();

% define algorithm options
Opts = [];

Opts.Q = diag([1,1]);                 % state weighting matrix
Opts.R = 0;                           % input weighting matrix
Opts.refTraj.Q = eye(2)./(0.2^2);     % state weighting matrix (ref. traj.)        
Opts.refTraj.R = 1/(14^2);            % input weighting matrix (ref. traj.) 
Opts.extHorizon.active = 1;           % use extended optimization horizon
Opts.extHorizon.horizon = 4;          % time steps for ext. horizon
Opts.extHorizon.decay = 'riseLinear'; % weight function for ext. horizon 

% offline phase computations
[obj,result] = convexInterpolationControl('cart',Param,Opts);

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