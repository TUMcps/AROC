% This script demonstrates the controller synthesis algorithm that is based
% on optimal control in generator space for a 2D quadrotor

% load benchmark parameters
Param = param_quadrotor2D();

% define algorithm options
Opts = [];

Opts.N = 10;                            % number of time steps
Opts.Ninter = 2;                        % number of intermediate steps
Opts.reachSteps = 20;                   % number of reachability steps
Opts.R = 1e-7*eye(2);                   % input weighting matrix
Opts.Q = diag([82,12.8,10,11,2,1]);     % state weighting matrix
Opts.extHorizon.active = true;          % use extended optimization horizon
Opts.extHorizon.decay = 'fall';         % weighting for extended horizon
Opts.extHorizon.horizon = 5;            % extended optimization horizon

% controller synthesis
clock = tic;
[objContr,res] = generatorSpaceControl('quadrotor2D',Param, Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s']);

% simulation 
[resSim,~,~] = simulateRandom(objContr);

% animation
speedUp = 0.5;
animate(resSim,'quadrotor2D',[],[],[],speedUp);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('x'); ylabel('z');

figure; hold on; box on
plotReach(res,[3,6],[.7 .7 .7]);
plotReachTimePoint(res,[3,6],'b');
plot(Param.R0,[3,4],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[3,6],'k');
xlabel('\phi'); ylabel('v_\phi');

figure; hold on; box on
plotReach(res,[4,5],[.7 .7 .7]);
plotReachTimePoint(res,[4,5],'b');
plot(Param.R0,[4,5],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[4,5],'k');
xlabel('v_x'); ylabel('v_z');