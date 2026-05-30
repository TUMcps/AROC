% This example demonstrates the linearization-based approach for computing 
% terminal regions of nonlinear discrete-time systems for the cart benchmark.

clear Param Opts

benchmark = 'cart';

% load parameters
Param = param_cart();

% algorithm settings: successive convexification for discrete-time systems
Opts.nonlinSysTermRegAlg = 'lin';
Opts.sysType = termRegSysType.DT;
Opts.integrator = discretizeDynamics.RK2;
Opts.timeStep = 0.1;
Opts.xEq = zeros(2,1);
Opts.uEq = 0;
Opts.termRegOrder = 5;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 10.0;
res = simulateRandom(terminalRegion,tFinal);

% visualization
figure; hold on; box on;
plotSimulation(res,[1,2],'k');
plot(terminalRegion.set,[1,2],'b');
xlabel('x_1'); ylabel('x_2');
