function T = computeTerminalRegionNonlinSysOpt(benchmark,Param,Opts)
%COMPUTETERMINALREGIONNONLINSYSOPT - computes a terminal region of nonlinear systems.
%
% Syntax:
%       T = computeTerminalRegionNonlinSysOpt(benchmark,Param,Opts)
%
% Description:
%       This function implements the iterative optimization-based
%       algorithms for computing the terminal region of nonlinear
%       sampled-data systems proposed in [1], [2], as well as the
%       disrete-time version of the algorithm in [1].
%
% Input Arguments:
%       -benchmark:     name of the considered benchmark model (see
%                       "aroc/benchmarks/...")
%
%       -Param:         a struct containing the benchmark parameters
%           .X:         set of admissible states
%                       (class: interval/zonotope/polytope)
%           .U:         set of admissible control inputs
%                       (class: interval/zonotope/polytope)
%           .W:         set of disturbances
%                       (class: zonotope)
%
%       -Opts:          a struct containing the algorithm settings
%           .xEq:                   equlibrium state to be stabilized
%                                   (double vector)
%           .uEq:                   equilibrium control input
%                                   (double vector)
%           .timeStep:              sampling time 
%                                   (positive scalar)
%           .termRegOrder:          (optional) order of the zonotope 
%                                   representing the terminal region
%                                   (positive scalar, >= 1)
%           .nonlinSysTermRegAlg:   ('lin' / 'poly')
%           -sysType:               type of dynamical system, e.g., 
%                                   sampled-data or discrete-time system 
%                                   (class: termRegSysType)
%           .integrator:            (optional) integration scheme for time-
%                                   discretizing the dynamical system - 
%                                   only considered if sysType == termRegSysType.DT
%                                   (class: discretizeDynamics)
%           .initialGuess:          (optional) struct storing parameters
%                                   for computing an initial guess
%               .Q:                 state weighting matrix
%                                   (positive-definite matrix of dimension
%                                   [Param.X.dim,Param.X.dim])
%               .R:                 control input weighting matrix
%                                   (positive-definite matrix of dimension
%                                   [Param.U.dim,Param.U.dim])
%               .weights:           struct storing the weighting factors 
%                                   for different options for computing the 
%                                   initial guess - valid field names:
%                                   'RiccatiEq', 'minRCI', 'minRPI'
%                                   'bwReachInputs'
%                                   (non-negative scalars)
%           .verbose:               if set to true, the algorithm prints 
%                                   information about each iteration to the 
%                                   command window. If set to false, no 
%                                   iterative output is shown. 
% 
% Output Arguments:
%       -T:     verified solution
%               (class: termRegNonlinSysOpt)
%
% See also:
%       -
%
% References:
%       [1] L. Schäfer et al., "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770.
%       [2] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169. 
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Synthesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% initialization ---------------------------------------------------------

Opts = checkOpts(Opts,'nonlinSysOpt',Param,Param.X.dim,Param.U.dim);

% instantiate object
switch Opts.nonlinSysTermRegAlg
    case 'lin'
        solver = computeTermRegNonlinSysLinApproach(Param,Opts);
    case 'poly'
            solver = computeTermRegNonlinSysPolyApproach(Param,Opts);
    otherwise
        error(sprintf(['No algorithm <%s> for computing ' ...
            'terminal regions of nonlinear systems implemented.']))
end

% initialize dynamical system for reachability analysis
dynamics = termRegNonlinSysDynamics(benchmark,Opts.sysType,...
    solver,Opts,Param);
% identify abstraction error arguments
solver = solver.getAbsErrArguments(dynamics);

% compute initial guess
initialGuess = solver.computeInitialGuess( ...
    dynamics,Opts.initialGuess.Q,Opts.initialGuess.R,Opts.initialGuess.weights);

% initialize trust region
solver.tr_set = zonotope(zeros(solver.nx,1), ...
    diag(0.05*(interval(initialGuess.set).sup-interval(initialGuess.set).inf)));

% initialize cost function coefficients
solver = solver.initCostFunction(initialGuess.set);

% main loop --------------------------------------------------------------

sol = cell(solver.maxIter+1,1);
sol{1} = initialGuess; 
auxData = cell(solver.maxIter+1,1);
auxData{1} = termRegNonlinSysSolutionAuxData( ...
    false,solver.costSet,0,interval(zeros(solver.nx,1)),0,0);
for iter = 1:solver.maxIter
    disp(['----- iteration ',num2str(iter),' -----'])
    % solve optimization problem
        [candSolver,candSol,candAuxData] ...
        = solver.solveOptProblem(dynamics,sol{iter},auxData{iter}.flagIsFeasible);
    % pick the better solution 
    if auxData{iter}.flagIsFeasible && ...
            (candAuxData.costSetVolume < auxData{iter}.costSetVolume)
        sol{iter+1} = sol{iter}; auxData{iter+1} = auxData{iter};
    else
        solver = candSolver; sol{iter+1} = candSol; auxData{iter+1} = candAuxData;
    end

    % evaluate convergence criterion
    dVolume = (auxData{iter+1}.costSetVolume - auxData{iter}.costSetVolume)/ ...
        auxData{iter}.costSetVolume;
    if solver.verbose
        disp(['solution feasible? ',num2str(auxData{iter+1}.flagIsFeasible)])
        disp(['volume of candidate terminal region: ',num2str(auxData{iter+1}.costSetVolume)])
        disp(['relative increase of volume: ',num2str(dVolume)])
    end
    % ... check satisfaction of convergence criterion
    if auxData{iter}.flagIsFeasible && dVolume <= solver.convergenceTolerance
        disp('----- Successive convexification/polynomialization algorithm converged! -----')
        break
    end

    % update trust region
    absErr_true = solver.computeAbsErr(dynamics,sol{iter+1}.set);
    [solver,tr_action] = solver.updateTrustRegion(auxData{iter+1},absErr_true);
    if solver.verbose
        disp(['trust region update: ',tr_action])
    end
end
sol = sol(1:iter+1);

% verification (see [1, Sec. III 2]) -------------------------------------

disp('--------------- Verifying safety .... ---------------')
[flagVerified,termReg_verified] = solver.verifyTerminalRegion(dynamics,sol{end});

if flagVerified
    disp('--------- Terminal region verified as safe! ---------')
    T = termReg_verified;
else
    warning('Failed to compute a (safe) terminal region!')
    T = termRegNonlinSysOpt.empty;
end

end
