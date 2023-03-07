% This script demonstrates the controller synthesis algorithm that is based
% on optimal control in generator space on a lane change maneuver for an
% autonomous truck

% load benchmark parameters
Param = param_truck();

% define algorithm options
Opts = [];

Opts.N = 30;                         % number of time steps
Opts.Ninter = 5;                     % number of intermediate time steps
Opts.reachSteps = 20;                % number of reachability steps
Opts.Q = diag([1,2,1,1,10,10]);      % state weighting matrix
Opts.R = 1e-6*eye(2);                % input weighting matrix

% controller synthesis
clock = tic;
[objContr,res] = generatorSpaceControl('truck',Param, Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s']);

% simulation 
[resSim,~,~] = simulateRandom(objContr);

% animation
lanelets = {};
lanelets{1} = polygon([-100,-100,100,100],[1.84 -1.84 -1.84 1.84]);
lanelets{2} = polygon([-100,-100,100,100],[1.84 5.52 5.52 1.84]);

animate(resSim,'truck',[],[],[],[],lanelets);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('\delta'); ylabel('\phi');

figure; hold on; box on
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plot(Param.R0,[3,4],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[3,4],'k');
xlabel('\alpha'); ylabel('v');

figure; hold on; box on
plotReach(res,[5,6],[.7 .7 .7]);
plotReachTimePoint(res,[5,6],'b');
plot(Param.R0,[5,6],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[5,6],'k');
xlabel('x'); ylabel('y');