% This script demonstrates how to compute a terminal region for the
% platoon benchmark using the zonotope approach for linear systems

% load system parameter
Param = param_platoon();

Param = rmfield(Param,'R0');
Param = rmfield(Param,'xf');
Param = rmfield(Param,'tFinal');

% define algorithm settings
Opts = [];

Opts.timeStep = 0.1;                        % time step
Opts.xEq = [0; 0; 10; 0; 10; 0; 10; 0];     % equilibrium point
Opts.uEq = [0; 0; 0; 0];                    % control input eq. point
Opts.costFun = 'none';                      % cost fun. opt. problem

% compute terminal region
clock = tic;
T = computeTerminalRegion('platoon','zonoLinSys',Param,Opts);
tComp = toc(clock);

disp(['Computation time: ',num2str(tComp),'s']);

% simulate terminal region controller
tFinal = 10;
res = simulateRandom(T,tFinal);

% visualization
figure; hold on; box on;
plotSimulation(res,[3,4],'k');
plot(T.set,[3,4],'b');
xlabel('x_3'); ylabel('x_4');