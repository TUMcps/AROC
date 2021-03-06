% This script demonstrates the control algorithm that is based on the
% optimization over reachable sets for the turn right maneuver of a car

% load benchmark parameter
Param = param_car_turnRight();

% define algorithm options
Opts = settings_optBasedContr_car();

% controller synthesis
clock = tic;
[objContr,res] = optimizationBasedControl('car',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s']);

% simulation 
[res,~,~] = simulateRandom(objContr,res,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(res,[1,2],'k');
xlabel('v');
ylabel('\phi');

figure; hold on; box on
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plot(Param.R0,[3,4],'w','Filled',true);
plotSimulation(res,[3,4],'k');
xlabel('x');
ylabel('y');
