% Comparison of the linearization-based and the polynomialization-based 
% algorithm for computing terminal regions of nonlinear sampled-data systems.

clear Param Opts

benchmark = 'cart';

% load parameters
Param = param_cart();

% algorithm settings
% ... terminal region
Opts.timeStep = 0.1;
Opts.xEq = zeros(2,1);
Opts.uEq = 0;
Opts.termRegOrder = 5;
Opts.sysType = termRegSysType.SD;
% ... initial guess weights
Opts.initialGuess.weights.RiccatiEq = 0.0;
Opts.initialGuess.weights.minRCI = 1.0;
Opts.initialGuess.weights.minRPI = 0.0;

%% linearization-based approach
Opts.nonlinSysTermRegAlg = 'lin';
% compute terminal region
terminalRegionLin = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

%% polynomialization-based approach
Opts.nonlinSysTermRegAlg = 'poly';

% compute terminal region
terminalRegionPoly = computeTerminalRegion( ...
    benchmark,'nonlinSysOpt',Param,Opts);

% visualization
figure; hold on; box on;
plot(terminalRegionLin.set,[1,2],'b');
plot(terminalRegionPoly.set,[1,2],'r');
legend({'Linearization-based Approach', 'Polynomialization-based Approach'})
xlabel('x_1'); ylabel('x_2');
