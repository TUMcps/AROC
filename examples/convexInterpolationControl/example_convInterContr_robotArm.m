% This script demonstrates the differences between the scaled linear
% control law and the linear control law determined by solving an
% optimization problem on the robot arm benchmark.

% load benchmark parameter
Param = param_robotArm();


%% SCALED LINEAR CONTROL LAW

% define algorithm options
Opts = settings_convInterContr_robotArm();
Opts.controller = 'linear';
Opts.approx.method = 'scaled';

% controler synthesis
clock = tic;
[objContrScaled,resContrScaled] = ...
    convexInterpolationControl('robotArm',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (scaled linear controller): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[resContrScaled,~,~] = simulateRandom(objContrScaled,...
                                      resContrScaled,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resContrScaled,[1,3],[.7 .7 .7]);
plotReachTimePoint(resContrScaled,[1,3],'b');
plot(Param.R0,[1,3],'w','Filled',true);
plotSimulation(resContrScaled,[1,3],'k');
xlabel('$\theta_1$','interpreter','latex');
ylabel('$\dot\theta_1$','interpreter','latex');
title('Scaled Linear Control Law');

figure; hold on; box on
plotReach(resContrScaled,[2,4],[.7 .7 .7]);
plotReachTimePoint(resContrScaled,[2,4],'b');
plot(Param.R0,[2,4],'w','Filled',true);
plotSimulation(resContrScaled,[2,4],'k');
xlabel('$\theta_2$','interpreter','latex');
ylabel('$\dot\theta_2$','interpreter','latex');
title('Scaled Linear Control Law');



%% OPTIMIZED LINEAR CONTROL LAW

% define algorithm options
Opts = settings_convInterContr_robotArm();
Opts.controller = 'linear';
Opts.approx.method = 'optimized';
Opts.approx.lambda = 0.5;

% controller synthesis
clock = tic;
[objContrOptimized,resContrOptimized] = ...
    convexInterpolationControl('robotArm',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (optimized linear controller): ', ...
      num2str(tComp),'s']);

% simulation 
[resContrOptimized,~,~] = simulateRandom(objContrOptimized,....
                                         resContrOptimized,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resContrOptimized,[1,3],[.7 .7 .7]);
plotReachTimePoint(resContrOptimized,[1,3],'r');
plot(Param.R0,[1,3],'w','Filled',true);
plotSimulation(resContrOptimized,[1,3],'k');
xlabel('$\theta_1$','interpreter','latex');
ylabel('$\dot\theta_1$','interpreter','latex');
title('Optimized Linear Control Law');

figure; hold on; box on
plotReach(resContrOptimized,[2,4],[.7 .7 .7]);
plotReachTimePoint(resContrOptimized,[2,4],'r');
plot(Param.R0,[2,4],'w','Filled',true);
plotSimulation(resContrOptimized,[2,4],'k');
xlabel('$\theta_2$','interpreter','latex');
ylabel('$\dot\theta_2$','interpreter','latex');
title('Optimized Linear Control Law');