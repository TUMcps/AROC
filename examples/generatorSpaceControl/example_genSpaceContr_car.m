% This script demonstrates the controller synthesis algorithm that is based
% on optimal control in generator space on the turn right maneuver of a car

% load benchmark parameters
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 5;                         % number of time steps
Opts.Ninter = 5;                    % number of intermediate time steps
Opts.extHorizon.active = 1;         % use extended optimization horizon
Opts.extHorizon.horizon = 5;        % time steps for ext. horizon
Opts.extHorizon.decay = 'fall';     % weight function for ext. horizon   

% controller synthesis
clock = tic;
[objContr,res] = generatorSpaceControl('car',Param, Opts);
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