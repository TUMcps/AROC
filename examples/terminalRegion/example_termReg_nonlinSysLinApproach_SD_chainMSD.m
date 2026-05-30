% This example demonstrates the linearization-based approach for computing 
% terminal regions of nonlinear sampled-data systems for a chain of 
% 10 coupled mass-spring-damper systems.

clear Param Opts

benchmark = 'chainMSD_10';
nMass = 10;

% load parameters
Param = param_chainMSD(nMass);

% algorithm settings
Opts.nonlinSysTermRegAlg = 'lin';
Opts.sysType = termRegSysType.SD;
Opts.timeStep = 0.1;
Opts.xEq = zeros(2*nMass,1);
Opts.uEq = zeros(nMass,1);
Opts.termRegOrder = 5;
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.0;
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 1.0;
Opts.initialGuess.weights.bwReachInputs = 0.5;

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
plotSimulation(res,[19,20],'k');
plot(terminalRegion.set,[19,20],'b');
xlabel('x_{19}'); ylabel('x_{20}'); 