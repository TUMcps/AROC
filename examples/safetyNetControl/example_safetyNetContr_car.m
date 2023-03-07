% This script demonstrates the performance of the Safety Net Controller on
% the example of a turn-right maneuver of the car benchmark

% load benchmark parameter
Param = param_car();


%% LQR COMFORT CONTROLLER

% define algorithm options
Opts = [];

Opts.reachSteps = 5;            % number of reachability steps
Opts.tComp = 0.05;              % allocated computation time 
Opts.refTraj.Q = 10*eye(4);     % state weighting matrix (reference traj.)        
Opts.refTraj.R = 1/10*eye(2);   % input weighting matrix (reference traj.)
    
% settings for the comfort controller that is applied online
Opts.controller = {'LQR','LQR'};

Q = diag([0.2 10 31.2 1]);
contrOpts1.R = diag([50,170]); contrOpts1.Q = Q;
contrOpts2.R = diag([60,200]); contrOpts2.Q = Q;

Opts.contrOpts = {contrOpts1,contrOpts2};

% controller synthesis
[objContr,res] = safetyNetControl('car',Param,Opts);

% simulation
[resSim,~,~] = simulateRandom(objContr);

% visualization
figure; hold on; box on;
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(resSim,[1,2],'k');
xlabel('v'); ylabel('\phi');
title('LQR comfort controller');

figure; hold on; box on
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plotSimulation(resSim,[3,4],'k');
xlabel('x'); ylabel('y');
title('LQR comfort controller');


%% MPC COMFORT CONTROLLER

% settings for the comfort controller that is applied online
Opts.controller = 'MPC';
contrOpts.Q = diag([12 10 60 35]);
contrOpts.R = diag([0.1 1]);
contrOpts.horizon = 3;
contrOpts.Ninter = 2;
Opts.contrOpts = contrOpts;

% controller synthesis
[objContr,res] = safetyNetControl('car',Param,Opts);

% simulation
[resSim,~,~] = simulateRandom(objContr);

% visualization
figure; hold on; box on;
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(resSim,[1,2],'k');
xlabel('v'); ylabel('\phi');
title('MPC comfort controller');

figure; hold on; box on;
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plotSimulation(resSim,[3,4],'k');
xlabel('x'); ylabel('y');
title('MPC comfort controller');