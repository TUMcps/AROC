% This script demonstrates how to compute a terminal region for the
% random double integrator benchmark using the linSys method

clear;
yalmip('clear');

rng(123456);

N = 10;
Nsimulation = 10;
tol_verification = 1e-2;

% load system parameter
benchmark = 'doubleIntegrator';

Param = param_doubleIntegrator();

nx = 2; % state dimension

% set of state constraints
xWidth = ones(nx, 1);
Param.X = diag([10 4])*interval(-xWidth, xWidth);

% Options
% general options
Opts.N = N;
Opts.timeStep = 0.1;

% initial set
Opts.method = 'spherical';
Opts.nGenerators = 20;

Opts.taylorOrder = 5;

algorithm = 'linSys';
Opts.terminalRegionType = 'implicit';

Opts.terminalRegionSet = 'ellipsoid';
t_ell_implicit = tic;
T_ell_implicit = computeTerminalRegion(benchmark, algorithm, Param, Opts);
t_ell_implicit = toc(t_ell_implicit);
disp("Time elapsed for linSys with ellipsoids using the implicit method: " + num2str(t_ell_implicit))

Opts.terminalRegionSet = 'zonotope';
t_zono_implicit = tic;
T_zono_implicit = computeTerminalRegion(benchmark, algorithm, Param, Opts);
t_zono_implicit = toc(t_zono_implicit);
disp("Time elapsed for linSys with zonotopes using the implicit method: " + num2str(t_zono_implicit))

% Comparison with zonoLinSys
Opts.Tdomain = Param.X;                         % seach domain

Opts.terminalRegionType = 'fixed';

% compute terminal region
t_zono_fixed = tic;
T_zono_fixed = computeTerminalRegion('doubleIntegrator', algorithm, Param, Opts);
t_zono_fixed = toc(t_zono_fixed);
disp("Time elapsed for linSys with zonotopes using the fixed method: " + num2str(t_zono_fixed))

figure; hold on;

pX = plot(Param.X,[1 2],'k','DisplayName','State Constraints');

pell_implicit = plot(T_ell_implicit.set,[1 2],'b-','DisplayName','linSys - implicit - Ellipsoid');
pzono_implicit = plot(T_zono_implicit.set,[1 2],'r-','DisplayName','linSys - implicit - Zonotope');

pzonoLinSys = plot(T_zono_fixed.set, [1 2], 'g', 'DisplayName','linSys - fixed - Zonotope');

legend([pX pzonoLinSys pell_implicit pzono_implicit])

% Create simulations
t_simulations_ell_implicit = tic;
simulations_ell_implicit = T_ell_implicit.simulateRandom(Nsimulation, 'extreme');
t_simulations_ell_implicit = toc(t_simulations_ell_implicit);
t_simulations_zono_implicit = tic;
simulations_zono_implicit = T_zono_implicit.simulateRandom(Nsimulation, 'extreme');
t_simulations_zono_implicit = toc(t_simulations_zono_implicit);

t_simulations_zono_fixed = tic;
simulations_zono_fixed = T_zono_fixed.simulateRandom(Nsimulation, 'extreme');
t_simulations_zono_fixed = toc(t_simulations_zono_fixed);

% Check that all trajectories are correct

disp("Ellipsoid Implicit Verification...")
T_ell_implicit.verifyTrajectory(simulations_ell_implicit,tol_verification);
disp("Zonotope Implicit Verification...")
T_zono_implicit.verifyTrajectory(simulations_zono_implicit,tol_verification);

disp("Zonotope Fixed Verification...")
T_zono_fixed.verifyTrajectory(simulations_zono_fixed,tol_verification);