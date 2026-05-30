% This example demonstrates the polynomialization-based approach for 
% computing terminal regions of nonlinear sampled-data systems for the robot arm.

clear Param Opts

benchmark = 'robotArm';

% load parameters
Param = param_robotArm();

% algorithm settings
Opts.nonlinSysTermRegAlg = 'poly';
Opts.sysType = termRegSysType.SD;
Opts.timeStep = 0.1;
Opts.xEq = [pi/2;0;0;0]; 
Opts.uEq = zeros(2,1); 
Opts.termRegOrder = 5;

% compute terminal region
terminalRegion = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% simulate terminal region controller
tFinal = 10.0;
res = simulateRandom(terminalRegion,tFinal);

% visualization
tiledlayout(2,1)

nexttile; hold on; box on;
plot(terminalRegion.set,[1,3])
plotSimulation(res,[1,3],'k');
title('first joint')
xlabel('orientation')
ylabel('velocity')

nexttile; hold on; box on;
plot(terminalRegion.set,[2,4])
plotSimulation(res,[2,4],'k');
title('second joint')
xlabel('orientation')
ylabel('velocity')
