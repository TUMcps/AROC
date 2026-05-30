%% Test linSys

yalmip('clear');

% Params
% init
nu = 1; % input dimension
nx = 2; % state dimension

% set of admissible control inputs
uWidth = ones(nu, 1);
Param.U = interval(-uWidth, uWidth);

% set of uncertain disturbances (origin must be contained)
wWidth = 0.1*ones(nx, 1);
Param.W = interval(-wWidth, wWidth);

% set of state constraints
xWidth = ones(nx, 1);
Param.X = 10*interval(-xWidth, xWidth);

benchmark = 'doubleIntegrator';

% Options
% general options
Opts.N = 30;

Opts.verbose = true;
Opts.timeStep = 0.1;

% initial set
Opts.genMethod = 'spherical';
Opts.nGenerators = 20;

Opts.taylorOrder = 5;

algorithm = 'linSys';

% Starting with 'implicit' method
Opts.terminalRegionType = 'implicit';

Opts.terminalRegionSet = 'ellipsoid';
T_ell = computeTerminalRegion(benchmark, algorithm, Param, Opts);

Opts.terminalRegionSet = 'zonotope';
T_zono = computeTerminalRegion(benchmark, algorithm, Param, Opts);

% Then, testing 'fixed' method
Opts.terminalRegionType = 'fixed';

Opts.terminalRegionSet = 'zonotope';
T_zono_fixed = computeTerminalRegion(benchmark, algorithm, Param, Opts);

Nsimulation = 10;
% Create simulations
simulations_ell = T_ell.simulateRandom(Nsimulation, 'extreme');
simulations_zono = T_zono.simulateRandom(Nsimulation, 'extreme');

simulations_zono_fixed = T_zono_fixed.simulateRandom(Nsimulation, 'extreme');

T_ell.verifyTrajectory(simulations_ell)
T_zono.verifyTrajectory(simulations_zono)

T_zono_fixed.verifyTrajectory(simulations_zono_fixed)


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
Opts.numRef = 4;
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

%% Test NonlinSysOpt- linearization-based approach

benchmark = 'cart';

% load parameters
Param = param_cart();

% algorithm settings
% ... terminal region
Opts = [];
Opts.timeStep = 0.1;
Opts.xEq = zeros(2,1);
Opts.uEq = 0;
Opts.termRegOrder = 5;
Opts.sysType = termRegSysType.SD;
Opts.nonlinSysTermRegAlg = 'lin';
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.0;
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 0.0;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 10.0;
result = simulateRandom(terminalRegion,tFinal);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation are located inside the terminal
% region
res = checkFinalSimPointsInSet(result,terminalRegion.set);
assert(res == 1);

%% Test NonlinSysOpt - polynomialization-based approach

benchmark = 'cart';

% load parameters
Param = param_cart();

% algorithm settings
% ... terminal region
Opts = [];
Opts.timeStep = 0.1;
Opts.xEq = zeros(2,1);
Opts.uEq = 0;
Opts.termRegOrder = 5;
Opts.sysType = termRegSysType.SD;
Opts.nonlinSysTermRegAlg = 'poly';
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.0;
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 0.0;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 10.0;
result = simulateRandom(terminalRegion,tFinal);

% check if the input constraints are satisfied
res = checkSimInputs(result,Param.U);
assert(res == 1);

% check if the end point of the simulation are located inside the terminal
% region
res = checkFinalSimPointsInSet(result,terminalRegion.set);
assert(res == 1);
