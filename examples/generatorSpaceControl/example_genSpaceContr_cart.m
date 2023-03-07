% This script demonstrates on the cart benchmark model the influence
% of an extendet optimization horizon on the result.

% load benchmark parameter
Param = param_cart();


%% STANDARD CASE

% define algorithm options
Opts = [];

Opts.Q = diag([2,1]);               % state weighting matrix             
Opts.R = 0;                         % input weighting matrix
Opts.refTraj.Q = eye(2)./(0.2^2);   % state weighting matrix (ref. traj.)
Opts.refTraj.R = 1/(14^2);          % input weighting matrix (ref. traj.)

% controller synthesis
clock = tic;
[objContr,res] = generatorSpaceControl('cart',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (without extended horizon): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[resSim,~,~] = simulateRandom(objContr);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('x'); ylabel('v');
title('Standard Case');


%% EXTENDED OPTIMIZATION HORIZON

% adapt algorithm options
Opts.extHorizon.active = 1;             % use extended optimization horizon
Opts.extHorizon.horizon = 4;            % time steps for ext. horizon
Opts.extHorizon.decay = 'riseLinear';   % weight function for ext. horizon

% controller synthesis
clock = tic;
[objContrExtHorizon,resExtHorizon] = ...
    generatorSpaceControl('cart',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (with extended horizon): ', ...
      num2str(tComp),'s']);

% simulation 
[resSim,~,~] = simulateRandom(objContrExtHorizon);

% visualization
figure; hold on; box on
plotReach(resExtHorizon,[1,2],[.7 .7 .7]);
plotReachTimePoint(resExtHorizon,[1,2],'r');
plot(Param.R0,[1,2],'FaceColor','w','EdgeColor','k');
plotSimulation(resSim,[1,2],'k');
xlabel('x'); ylabel('v');
title('Extended Optimization Horizon');