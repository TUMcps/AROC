% This script demonstrates the control algorithm that is based on the
% optimization over reachable sets for the turn right maneuver of a car

% load benchmark parameter
Param = param_car();

% define algorithm options
Opts = [];

Opts.N = 10;                    % number of time steps  
Opts.reachSteps = 12;           % number of reachability steps
Opts.maxIter = 10;              % max number of iterations for optimization
Opts.bound = 10000;             % diff. bet. upper and lower bound of vars.
Opts.refTraj.Q = 10*eye(4);     % state weighting matrix (reference traj.)
Opts.refTraj.R = 1/10*eye(2);   % input weighting matrix (reference traj.)

% controller synthesis
clock = tic;
[objContr,res] = optimizationBasedControl('car',Param,Opts);
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