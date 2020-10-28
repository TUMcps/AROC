%% Test Reachset Model Predictive Control

% load benchmark parameter
Param = param_stirredTankReactor_traj2();

% define algorithm options
Opts = settings_reachsetMPC_stirredTankReactor_traj2();

% offline phase computations
result = reachsetMPC('stirredTankReactor',Param,Opts);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation is inside the terminal region
p = getFinalSimPoints(result);
res = in(Opts.termReg,p);
assert(res == 1);