% This script demonstrate how to compute a terminal region for the
% artificial system benchmark using the subpaving algorithm

% load system parameter
Param = param_artificialSystem();
Param = rmfield(Param,'x0');
Param = rmfield(Param,'xf');

% define algorithm settings
Opts = [];

Opts.Q = [1 0;0 1];                             % state weighting matrix
Opts.R = 1;                                     % input weighting matrix
Opts.Tdomain = interval([-3;-2],[3;2]);         % search domain
Opts.Tinit = interval([-0.1;-0.1],[0.1;0.1]);   % initial guess
Opts.xEq = [0;0];                               % equilibrium point
Opts.uEq = 0;                                   % control inp. eq. point
Opts.numRef = 5;                                % number of refinements
Opts.reachSteps = 100;                          % number of reach. steps
Opts.tMax = 100;                                % final time
Opts.enlargeFac = 1.5;                          % enlargement of domain

% compute terminal region
clock = tic;
T = computeTerminalRegion('artificialSystem','subpaving',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s', newline]);

% simulate terminal region controller
tFinal = Opts.tMax;
res = simulateRandom(T,tFinal);

% visualization
figure; hold on; box on;
for i = 1:length(T.subpaving)
   plot(T.subpaving{i},[1,2],'r'); 
end
plot(T.set,[1,2],'b');
xlabel('x_1'); ylabel('x_2');

figure; hold on; box on;
plot(T.set,[1,2],'b');
plotSimulation(res,[1,2],'k');
xlabel('x_1'); ylabel('x_2');