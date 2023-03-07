%% Test Model Predictive Control Linear Systems

% get benchmark parameter
Param = param_doubleIntegrator();

Param.xf = zeros(2,1);
Param.x0 = [3;-3];

% controller settings
Opts = [];

Opts.termReg = mptPolytope([1 0;0 1;-1 0;0 -1;1 1;-1 -1],[1;1;1;1;1.2;1.2]);
Opts.tOpt = 8;
Opts.N = 40;

% model predictive control
result = linSysMPC('doubleIntegrator',Param,Opts);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end points of the simulation are located inside the terminal
% region
res = checkFinalSimPointsInSet(result,Opts.termReg);
assert(res == 1);

% check if the final reachable set is located inside the terminal region
res = contains(Opts.termReg,result.reachSetTimePoint{end});
assert(res == 1);