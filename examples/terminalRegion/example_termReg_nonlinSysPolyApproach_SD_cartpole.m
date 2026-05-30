% This example demonstrates the polynomialization-based algorithm for
% computing terminal regions of nonlinear sampled-data sytems for the
% cartpole benchmark.

clear Param Opts

benchmark = 'cartpole';

% load system parameters
Param = param_cartpole();

% algorithm settings
Opts.timeStep = 0.1;
Opts.xEq = zeros(Param.X.dim,1);
Opts.uEq = zeros(Param.U.dim,1);
Opts.termRegOrder = 5;
Opts.nonlinSysTermRegAlg = 'poly';
Opts.sysType = termRegSysType.SD;
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.5; 
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 1.0;
Opts.initialGuess.weights.bwReachInputs = 0.5;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 10.0;
res = simulateRandom(terminalRegion,tFinal);

% visualization
tiledlayout(2,1)

nexttile; hold on; box on;
dim = [1,3];
plotSimulation(res,dim,'k');
plot(terminalRegion.set,dim,'b');
xlabel('x_1'); ylabel('x_3');

nexttile; hold on; box on;
dim = [2,4];
plotSimulation(res,dim,'k');
plot(terminalRegion.set,dim,'b');
xlabel('x_2'); ylabel('x_4');