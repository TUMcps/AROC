% This script demonstrates the controller synthesis algorithm using a
% polynomial control law on the turn right maneuver of a car

% load benchmark parameters
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 5;                 % number of time steps
Opts.Ninter = 5;            % number of intermediate time steps
Opts.ctrlOrder = 1;         % polynomial order of the controller
Opts.Q = diag([1,2,1,1]);   % state weighting matrix

% controller synthesis
clock = tic;
[objContr,res] = polynomialControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s']);

% simulation 
[resSim,~,~] = simulateRandom(objContr);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('v'); ylabel('\phi');

figure; hold on; box on
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plot(Param.R0,[3,4],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[3,4],'k');
xlabel('x'); ylabel('y');