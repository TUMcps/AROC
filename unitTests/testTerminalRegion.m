%% Test Zonotope Linear System

% load system parameter
Param = param_doubleIntegrator();

% define algorithm settings
Opts = [];

Opts.Tdomain = interval(-ones(2,1),ones(2,1));
Opts.timeStep = 0.1;
Opts.N = 30;
Opts.Q = eye(2);
Opts.R = eye(1);
Opts.maxDist = 1e-2;
Opts.genMethod = 'sampling2D';
Opts.costFun = 'sum';

% compute terminal region
T = computeTerminalRegion('doubleIntegrator','zonoLinSys',Param,Opts);

% simulate terminal region controller
tFinal = 10;
result = simulateRandom(T,tFinal);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the final simulation points are all located inside the terminal
% set
res = checkFinalSimPointsInSet(result,T.set);
assert(res == 1);


%% Test Subpaving

% load system parameter
Param = param_artificialSystem();
Param = rmfield(Param,'x0');
Param = rmfield(Param,'xf');

% define algorithm options
Opts = [];

Opts.Q = [1 0;0 1];
Opts.R = 1;
Opts.Tdomain = interval([-3;-2],[3;2]);
Opts.Tinit = interval([-0.1;-0.1],[0.1;0.1]);
Opts.xEq = [0;0];
Opts.uEq = 0;
Opts.numRef = 5;
Opts.reachSteps = 100;
Opts.tMax = 100;
Opts.enlargeFac = 1.5;

% compute terminal region
T = computeTerminalRegion('artificialSystem','subpaving',Param,Opts);

% simulate terminal region controller
tFinal = Opts.tMax;
result = simulateRandom(T,tFinal,10,0.5,0.1);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation are located inside the terminal
% region
res = checkFinalSimPointsInSet(result,T.set);
assert(res == 1);