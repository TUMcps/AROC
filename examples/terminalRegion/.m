% This example demonstrates the polynomialization-based approach for computing 
% terminal regions of nonlinear sampled-data systems for a chain of 
% 5 coupled mass-spring-damper systems.

clear Param Opts

benchmark = 'chainMSD_5';
nMass = 5;

% load parameters
Param = param_chainMSD(nMass);

% algorithm settings
Opts.nonlinSysTermRegAlg = 'poly';
Opts.sysType = termRegSysType.SD;
Opts.timeStep = 0.1;
Opts.xEq = zeros(2*nMass,1);
Opts.uEq = zeros(nMass,1);
Opts.termRegOrder = 5;
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.0;
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 0.1;
Opts.initialGuess.weights.bwReachInputs = 0.5;

Opts.verbose = true;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 2.0;
res = simulateRandom(terminalRegion,tFinal);

% visualization
tiledlayout(2,1)

nexttile; hold on; box on;
plotSimulation(res,[1,2],'k');
plot(terminalRegion.set,[1,2],'b');
xlabel('x_1'); ylabel('x_2'); 

nexttile; hold on; box on;
plotSimulation(res,[9,10],'k');
plot(terminalRegion.set,[9,10],'b');
xlabel('x_{9}'); ylabel('x_{10}'); 