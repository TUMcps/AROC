% This script demonstrates the differences between the exact control law 
% and the linear approximation for the turn right maneuver of the car
% benchmark

% load benchmark parameter
Param = param_car();


%% EXACT CONTROLLER

% define algorithm options
Opts = [];

Opts.Q = diag([2,5,1,1]);             % state weighting matrix
Opts.R = diag([0;0]);                 % input weighting matrix
Opts.refTraj.Q = 10*eye(4);           % state weighting matrix (ref. traj.)
Opts.refTraj.R = 1/10*eye(2);         % input weighting matrix (ref. traj.)
Opts.controller = 'exact';            % controller

% controller synthesis
clock = tic;
[objContrExact,resExact] = convexInterpolationControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (exact controller): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[resSim,~,~] = simulateRandom(objContrExact);

% visualization
figure; hold on; box on
plotReach(resExact,[1,2],[.7 .7 .7]);
plotReachTimePoint(resExact,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('v'); ylabel('\phi');
title('Exact Controller');

figure; hold on; box on
plotReach(resExact,[3,4],[.7 .7 .7]);
plotReachTimePoint(resExact,[3,4],'b');
plot(Param.R0,[3,4],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[3,4],'k');
xlabel('x'); ylabel('y');
title('Exact Controller');


%% LINEAR CONTROLLER

% adapt algorithm options
Opts.controller = 'linear';

% controller synthesis
clock = tic;
[objContrLin,resLin] = convexInterpolationControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (linear controller): ', ...
      num2str(tComp),'s']);

% simulation 
[resSim,~,~] = simulateRandom(objContrLin);

% visualization
figure; hold on; box on
plotReach(resLin,[1,2],[.7 .7 .7]);
plotReachTimePoint(resLin,[1,2],'r');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('v'); ylabel('\phi');
title('Linear Controller');

figure; hold on; box on
plotReach(resLin,[3,4],[.7 .7 .7]);
plotReachTimePoint(resLin,[3,4],'r');
plot(Param.R0,[3,4],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[3,4],'k');
xlabel('x'); ylabel('y');
title('Linear Controller');