%% Test Linear Control Law

% get benchmark parameter
Param = param_car_turnRight();

% define algorithm options
Opts = settings_convInterContr_car();
Opts.controller = 'linear';

% offline phase computations
[obj,result] = convexInterpolationControl('car',Param,Opts);

% simulation 
[result,~,~] = simulateRandom(obj,result,10,0.5,0.6,2);

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
Param = param_car_turnRight();

% define algorithm options
Opts = settings_convInterContr_car();
Opts.controller = 'exact';
Opts.polyZono.N = 5;

% offline phase computations
[obj,result] = convexInterpolationControl('car',Param,Opts);

% simulation 
[result,~,~] = simulateRandom(obj,result,10,0.5,0.6,2);

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
Param = param_doubleIntegrator();

% define algorithm options
Opts = settings_convInterContr_doubleIntegrator();
Opts.controller = 'linear';
Opts.extHorizon.active = 1;
Opts.extHorizon.horizon = 4;
Opts.extHorizon.decay = 'uniform';

% offline phase computations
[obj,result] = convexInterpolationControl('doubleIntegrator',Param,Opts);

% simulation 
[result,~,~] = simulateRandom(obj,result,10,0.5,0.6,2);

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