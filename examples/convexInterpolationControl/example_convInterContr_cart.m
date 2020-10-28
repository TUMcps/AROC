% This script demonstrates on the cart benchmark model the influence of
% different weighting functions on the final result for the case that an
% extendet horizon is used.

% load benchmark parameter
Param = param_cart();


%% STANDARD CASE

% define algorithm options
Opts = settings_convInterContr_cart();

% controller synthesis
clock = tic;
[objContr,res] = convexInterpolationControl('cart',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (without extended horizon): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[res,~,~] = simulateRandom(objContr,res,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(res,[1,2],'k');
xlabel('x');
ylabel('v');
title('Standard Case');


%% EXTENDED OPTIMIZATION HORIZON

% define algorithm options
Opts = settings_convInterContr_cart();
Opts.extHorizon.active = 1;
Opts.extHorizon.horizon = 4;
Opts.extHorizon.decay = 'riseLinear';

% controller synthesis
clock = tic;
[objContrExtHorizon,resExtHorizon] = ...
    convexInterpolationControl('cart',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (with extended horizon): ', ...
      num2str(tComp),'s']);

% simulation 
[resExtHorizon,~,~] = simulateRandom(objContrExtHorizon,resExtHorizon,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resExtHorizon,[1,2],[.7 .7 .7]);
plotReachTimePoint(resExtHorizon,[1,2],'r');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(resExtHorizon,[1,2],'k');
xlabel('x');
ylabel('v');
title('Extended Optimization Horizon');