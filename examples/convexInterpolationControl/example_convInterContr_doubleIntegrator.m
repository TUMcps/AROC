% This script demonstrates on the double-integrator benchmark the influence 
% of different weighting functions on the final result for the case that an
% extendet horizon is used.

% load benchmark parameter
Param = param_doubleIntegrator();


%% UNIFORM WEIGHTING FUNCTION

% define algorithm options
Opts = settings_convInterContr_doubleIntegrator();
Opts.extHorizon.active = 1;
Opts.extHorizon.horizon = 4;
Opts.extHorizon.decay = 'uniform';

% controller synthesis
clock = tic;
[objContrUniform,resUniform] = ...
    convexInterpolationControl('doubleIntegrator',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (uniform weighting): ', ...
      num2str(tComp),'s', newline]);

% simulation 
[resUniform,~,~] = simulateRandom(objContrUniform,resUniform,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resUniform,[1,2],[.7 .7 .7]);
plotReachTimePoint(resUniform,[1,2],'b');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(resUniform,[1,2],'k');
xlabel('x');
ylabel('v');
title('Uniform Weigthing Function');


%% END-ONLY WEIGHTING FUNCTION

% define algorithm options
Opts = settings_convInterContr_doubleIntegrator();
Opts.extHorizon.active = 1;
Opts.extHorizon.horizon = 4;
Opts.extHorizon.decay = 'end';

% controller synthesis
clock = tic;
[objContrEndOnly,resEndOnly] = ...
    convexInterpolationControl('doubleIntegrator',Param,Opts);
tComp = toc(clock);

disp([newline,'Computation time (end-only weighting): ', ...
      num2str(tComp),'s']);

% simulation 
[resEndOnly,~,~] = simulateRandom(objContrEndOnly,resEndOnly,10,0.5,0.6,2);

% visualization
figure; hold on; box on
plotReach(resEndOnly,[1,2],[.7 .7 .7]);
plotReachTimePoint(resEndOnly,[1,2],'r');
plot(Param.R0,[1,2],'w','Filled',true);
plotSimulation(resEndOnly,[1,2],'k');
xlabel('x');
ylabel('v');
title('End-Only Weigthing Function');