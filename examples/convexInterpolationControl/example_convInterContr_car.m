% This script demonstrates the differences between the exact control law 
% and the linear approximation for the turn right maneuver of the car
% benchmark

% load benchmark parameter
Param = param_car_turnRight();


%% EXACT CONTROLLER

% define algorithm options
Opts = settings_convInterContr_car();
Opts.controller = 'exact';

% controller synthesis
clock = tic;
[objContrExact,resExact] = convexInterpolationControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (exact controller): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[resExact,~,~] = simulateRandom(objContrExact,resExact,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resExact,[1,2],[.7 .7 .7]);
plotReachTimePoint(resExact,[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(resExact,[1,2],'k');
xlabel('v');
ylabel('\phi');
title('Exact Controller');

figure; hold on; box on
plotReach(resExact,[3,4],[.7 .7 .7]);
plotReachTimePoint(resExact,[3,4],'b');
plot(Param.R0,[3,4],'w','Filled',true);
plotSimulation(resExact,[3,4],'k');
xlabel('x');
ylabel('y');
title('Exact Controller');


%% LINEAR CONTROLLER

% define algorithm options
Opts = settings_convInterContr_car();
Opts.controller = 'linear';

% controller synthesis
clock = tic;
[objContrLin,resLin] = convexInterpolationControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (linear controller): ', ...
      num2str(tComp),'s']);

% simulation 
[resLin,~,~] = simulateRandom(objContrLin,resLin,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resLin,[1,2],[.7 .7 .7]);
plotReachTimePoint(resLin,[1,2],'r');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(resLin,[1,2],'k');
xlabel('v');
ylabel('\phi');
title('Linear Controller');

figure; hold on; box on
plotReach(resLin,[3,4],[.7 .7 .7]);
plotReachTimePoint(resLin,[3,4],'r');
plot(Param.R0,[3,4],'w','Filled',true);
plotSimulation(resLin,[3,4],'k');
xlabel('x');
ylabel('y');
title('Linear Controller');