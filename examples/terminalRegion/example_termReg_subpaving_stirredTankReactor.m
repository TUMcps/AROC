% This script demonstrate how to compute a terminal region for the
% stirred tank reactor benchmark using the subpaving algorithm

% load system parameter
Param = param_stirredTankReactor();
Param = rmfield(Param,'xf');
Param = rmfield(Param,'x0');

% define algorithm options
Opts = [];

Opts.Q = [1e5 0;0 1];                           % state weighting matrix
Opts.R = 1;                                     % input weighting matrix
Opts.Tdomain = interval([-0.07;-6],[0.07;5]);   % search domain
Opts.Tinit = interval([-0.05;-3],[0.05;3]);     % initial guess
Opts.xEq = [0;0];                               % equilibrium point
Opts.uEq = 0;                                   % control inp. eq. point
Opts.numRef = 5;                                % number of refinements
Opts.reachSteps = 50;                           % number of reach. steps
Opts.tMax = 50;                                 % final time
Opts.enlargeFac = 1.1;                          % enlargement factor

% compute terminal region
clock = tic;
T = computeTerminalRegion('stirredTankReactor','subpaving',Param,Opts);
tComp = toc(clock);

disp(['Computation time: ',num2str(tComp),'s']);

% simulate terminal region controller
tFinal = Opts.tMax;
res = simulateRandom(T,tFinal);

% visualization
figure; hold on; box on;
for i = 1:length(T.subpaving)
   plot(T.subpaving{i},[1,2],'r'); 
end
plot(T.set,[1,2],'b');
xlabel('C_A'); ylabel('T');

figure; hold on; box on;
plot(T.set,[1,2],'b');
plotSimulation(res,[1,2],'k');
xlabel('C_A'); ylabel('T');