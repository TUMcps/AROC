% This script demonstrates the Reachset Model Predictive Control algorithm
% for the stirred tank reactor benchmark

% load benchmark parameter
Param = param_stirredTankReactor_traj2();

% define algorithm options
Opts = settings_reachsetMPC_stirredTankReactor_traj2();

% execute model predictive control algorithm
res = reachsetMPC('stirredTankReactor',Param,Opts);

% visualization
figure; hold on; box on
plot(Opts.termReg,[1,2],'FaceColor',[100 182 100]./255,...
     'EdgeColor','none','FaceAlpha',0.8,'Filled',true);
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(res,[1,2],'r');
xlabel('C_A');
ylabel('T');
xlim([-0.35,0.05]);
ylim([-35,5]);