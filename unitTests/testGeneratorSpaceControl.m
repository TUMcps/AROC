%% Test Generator Space Control

% get benchmark parameter
Param = param_car_turnRight();

% define algorithm options
Opts = settings_genSpaceContr_car();
Opts = rmfield(Opts,'extHorizon');

% offline phase computations
[obj,result] = generatorSpaceControl('car',Param,Opts);

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

% get benchmark parameter
Param = param_car_turnRight();

% define algorithm options
Opts = settings_genSpaceContr_car();

Opts.extHorizon.active = 1;
Opts.extHorizon.horizon = 3;
Opts.extHorizon.decay = 'fall';

% offline phase computations
[obj,result] = generatorSpaceControl('car',Param,Opts);

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