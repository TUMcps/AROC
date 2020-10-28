%% Test Linear System

% get benchmark parameter
Param = param_platoon();

% define algorithm options
Opts = settings_optBasedContr_platoon();

% offline phase computations
[objContr,res] = optimizationBasedControl('platoon',Param,Opts);

% simulation 
[result,~,~] = simulateRandom(objContr,res,10,0.5,0.6,2);

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
Param = param_car_turnRight();

% define algorithm options
Opts = settings_optBasedContr_car();

% offline phase computations
[objContr,res] = optimizationBasedControl('car',Param,Opts);

% simulation 
[result,~,~] = simulateRandom(objContr,res,10,0.5,0.6,2);

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