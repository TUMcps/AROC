%% Test Generator Space Control

% get benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];
Opts.N = 10;                        % number of time steps
Opts.Ninter = 5;                    % number of intermediate time steps

% offline phase computations
[obj,result] = generatorSpaceControl('car',Param,Opts);

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

% get benchmark parameter
Param = param_car();

% adapt algorithm options
Opts.extHorizon.active = 1;         % use extended optimization horizon
Opts.extHorizon.horizon = 3;        % time steps for ext. horizon
Opts.extHorizon.decay = 'fall';     % weight function for ext. horizon  

% offline phase computations
[obj,result] = generatorSpaceControl('car',Param,Opts);

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