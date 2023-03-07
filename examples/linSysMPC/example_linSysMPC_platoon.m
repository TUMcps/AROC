% This script demonstrates linear model predictive control for the
% platoon benchmark

% load system parameter
ParamTerm = param_platoon();

ParamTerm = rmfield(ParamTerm,'R0');
ParamTerm = rmfield(ParamTerm,'xf');
ParamTerm = rmfield(ParamTerm,'tFinal');

Param = ParamTerm;
Param.xf = [0; 0; 10; 0; 10; 0; 10; 0];
Param.x0 = [6; 2; 16; 2; 16; 2; 16; 2];

Param.W = 0.05*Param.W;

% compute terminal region
OptsTerm = [];

OptsTerm.timeStep = 0.1;
OptsTerm.xEq = Param.xf;
OptsTerm.uEq = [0; 0; 0; 0];
OptsTerm.costFun = 'none';

T = computeTerminalRegion('platoon','zonoLinSys',ParamTerm,OptsTerm);

% controller settings
Opts = [];

Opts.termReg = T;           % terminal region
Opts.tOpt = 2;              % prediction horizon
Opts.N = 10;                % number of time steps
Opts.realTime = true;       % require that computations run in real time
Opts.R = 0.01*eye(4);       % input weighting matrix (MPC)
Opts.Rlqr = 0.01*eye(4);    % input weighitng matrix (feedback controller)

% model predictive control
res = linSysMPC('platoon',Param,Opts);

% visualization
figure; hold on; box on;
plot(Opts.termReg.set,[3,4],'FaceColor',[100 182 100]./255,...
     'EdgeColor','none','FaceAlpha',0.8);
plotReach(res,[3,4],[.7 .7 .7]);
plotReachTimePoint(res,[3,4],'b');
plotSimulation(res,[3,4],'r');
xlabel('x_3'); ylabel('x_4');

figure; hold on; box on;
plot(res.simulation{1}.t,res.simulation{1}.u(:,1));
plot(res.simulation{1}.t,res.simulation{1}.u(:,2));
plot(res.simulation{1}.t,res.simulation{1}.u(:,3));
plot(res.simulation{1}.t,res.simulation{1}.u(:,4));
xlabel('time'); ylabel('control inputs');