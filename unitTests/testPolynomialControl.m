%% Test Polynomial Control

% load benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 5;                 % number of time steps
Opts.Ninter = 5;            % number of intermediate time steps
Opts.ctrlOrder = 1;         % polynomial order of the controller
Opts.Q = diag([1,2,1,1]);   % state weighting matrix

% controller synthesis
[objContr,result] = polynomialControl('car',Param,Opts);

% simulation 
[tmp,~,~] = simulateRandom(objContr);
add(result,tmp);

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