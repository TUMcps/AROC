% This script demonstrates the Reachset Model Predictive Control algorithm
% for the artificial system benchmark

% load benchmark parameter
Param = param_artificialSystem_traj1();

% define algorithm settings
Opts = settings_reachsetMPC_artificialSystem_traj1();

% execute reachset model predictive control algorithm
res = reachsetMPC('artificialSystem',Param,Opts);

% visualization
figure; hold on; box on
plot(Opts.termReg,[1,2],'FaceColor',[100 182 100]./255,...
     'EdgeColor','none','FaceAlpha',0.8,'Filled',true);
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(res,[1,2],'r');
xlabel('$x_1$','interpreter','latex');
ylabel('$x_2$','interpreter','latex');
xlim([-0.6,0.8]); ylim([-0.9,-0.5]);