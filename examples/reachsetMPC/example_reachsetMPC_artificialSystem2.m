% This script demonstrates the Reachset Model Predictive Control algorithm
% for the artificial system benchmark

% load benchmark parameter
Param = param_artificialSystem();
Param.x0 = [3.5;-2.5];

% define algorithm settings
Opts = [];

Opts.N = 18;                    % number of time steps
Opts.reachSteps = 5;            % number of reachability steps
Opts.tOpt = 150;                % final time for optimization horizon
Opts.Q = diag([3,3]);           % state weighting matrix  
Opts.R = 0.1;                   % input weighting matrix
Opts.Qlqr = diag([10;10]);      % state weighting matrix (feedback control)
Opts.Rlqr = 0.2;                % input weighting matrix (feedback control)
Opts.scale = 0.75;              % scaling factor for tightend input set
Opts.tComp = 2.5;               % allocated computation time
Opts.alpha = 0.1;               % contraction rate
Opts.maxIter = 12;              % max number of iterations for opt. control

% terminal region (see Sec.5 in Yu et al. (2013): "Tube MPC scheme based on 
% robust control invariant set with application to Lipschitz nonlinear 
% systems", Systems & Control Letters)
E = ellipsoid(inv([7.9997 -12.2019; -12.2019 27.0777]./10));
Z = zonotope(E,10,'i:norm:bnd');

Opts.termReg = mptPolytope(Z);

% execute reachset model predictive control algorithm
res = reachsetMPC('artificialSystem',Param,Opts);

% visualization
figure; hold on; box on
plot(Opts.termReg,[1,2],'FaceColor',[100 182 100]./255,...
     'EdgeColor','none','FaceAlpha',0.8);
plotReach(res,[1,2],[.7 .7 .7]);
plotReachTimePoint(res,[1,2],'b');
plotSimulation(res,[1,2],'r');
xlabel('x_1'); ylabel('x_2');
xlim([-4;4]); ylim([-5;2]);