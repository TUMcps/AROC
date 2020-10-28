% This script demonstrates the control algorithm that is based on the
% optimization over reachable sets for the platoon benchmark

% load benchmark parameter
Param = param_platoon();

% define algorithm options
Opts = settings_optBasedContr_platoon();

% controller synthesis
clock = tic;
[objContr,res] = optimizationBasedControl('platoon',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time: ',num2str(tComp),'s']);

% simulation 
[res,~,~] = simulateRandom(objContr,res,10,0.5,0.6,2);

% compute shifted final reachable set
Rshift = res.reachSetTimePoint{end};
Rshift = zonotope([center(Param.R0),generators(Rshift)]);

% visualization (reachable set projecton onto x_1-x_2-plane)
figure; hold on; box on
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(res,[1,2],'k');
xlabel('$x_1 [m]$','Interpreter','latex');
ylabel('$x_2 [\frac{m}{s}]$','Interpreter','latex');

% visualization (shifted final reachble set)
figure;
dims = {[1,2],[3,4],[5,6],[7,8]};

for i = 1:length(dims)
    subplot(2,2,i); hold on; box on; dim = dims{i};
    plot(Param.R0,dim,'r');
    plot(Rshift,dim,'b');
    xlabel(['$x_{',num2str(dim(1)),'} [m]$'],'Interpreter','latex');
    ylabel(['$x_{',num2str(dim(2)),'} [\frac{m}{s}]$'], ...
           'Interpreter','latex');
    if i == 1
        xlim([-0.3,0.3]); ylim([19.7,20.3]);
    else
        xlim([0.7,1.3]); ylim([-0.3,0.3]);        
    end
end