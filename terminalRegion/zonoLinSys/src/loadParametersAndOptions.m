function [Param, Opts] = loadParametersAndOptions(benchmark,Param,Opts)
% LOADPARAMETERSANDOPTIONS - load parameters and options
%
% Syntax:
%       [Param,Opts] = LOADPARAMETERSANDOPTIONS(benchmark,Param,Opts)
%
% Description:
%       This function loads the the benchmark parameters and algorithm
%       options. Addtionally, it augments the options structure with
%       additional properties that are needed for subsequent computations.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -Param:        struct containing benchmark parameters
%       -Opts:         struct containing algorithm settings
%
% Output Arguments:
%
%       -Param:  augmented struct containing the benchmark parameters
%       -Opts:   augmented struct containing the algorithm setting
%
% See Also:
%       computeTermRegZonoLinSys, checkParam, checkOpts
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
% Authors:      Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % get number of states, inputs, and disturbances
    [dynamics,nx,nu,nw] = computeDynamics(benchmark);

    % check benchmark parameters
    Param = checkParam(Param,'zonoLinSys',nx,nu,nw);

    % check algorithm settings
    Opts = checkOpts(Opts,'zonoLinSys',Param,nx,nu);

    % augment algorithm options structure with additional new options
    [Opts,Param] = augmentOpts(benchmark, Param, Opts);
    Opts.dynamics = dynamics;
end


% Auxiliary Functions -----------------------------------------------------

function [Opts,Param] = augmentOpts(benchmark, Param, Opts)
% precompute values required for the algorithm and store them in the Opts
% struct

    % additional algorithm settings
    Opts.TpOrTi = 'Ti';
    Opts.nrOfGenerators = 10; 
    Opts.cora.reductionTechnique = 'pca';
    Opts.cora.timeStep = Opts.timeStep;

    % center disturbance set
    if ~contains(Param.W,zeros(dim(Param.W),1))
       warning(['The disturbance set Params.W does not contain the ', ...
             'origin! This might prevent convergence of the algorithm!']); 
    end
    
    % determine best available convex optimization solver
    Opts.solver = bestAvailableSolver(Opts);

    % compute dynamics
    [A,B,D,nx,nu] = computeLinearSystemMatrices(benchmark,Opts);
    
    Opts.nx = nx;
    Opts.nu = nu;
    Opts.nxPlusnu = nx + nu;
    
    % construct augmented system matrices (states + input)
    B_ = [B; zeros(nu)];
    A_ = [[A; zeros(nu, nx)], B_];
    D_ = blkdiag(eye(nx),zeros(nu));
    
    Opts.linSysOrig = linearSys(A_,[D; zeros(nu,size(D,2))]);
    
    % compute disturbance set for augmented system
    W = zonotope(D * Param.W);
    W_ = zonotope([W.Z; zeros(nu,size(W.Z, 2))]);

    % compute feedback matrix [X; K*X] for augmented system
    K = computeStateFeedbackMatrix(Opts, A, B);
    Opts.XKX = [[eye(nx); K], zeros(Opts.nxPlusnu, nu)];
    
    % initialize search domain if it is not given
    if isempty(Opts.Tdomain)
        if isfield(Param,'X')
            Opts.Tdomain = initSearchDomain(K,Opts.xEq,Opts.uEq, ...
                                                        Param.U,Param.X);
        else
            Opts.Tdomain = initSearchDomain(K,Opts.xEq,Opts.uEq,Param.U);
        end
    end
    
    % subtract equilibrium point from all state and input sets
    Opts.Tdomain = Opts.Tdomain - Opts.xEq;
    Param.U = Param.U - Opts.uEq;
    
    % consider measurement error
    Opts.KV = [];
    
    if isfield(Param,'V') && ~isempty(Param.V)
       KV = K * zonotope(Param.V);
       Opts.KV = KV.Z;
    end
    
    % adapt parameter for reachability analysis to augmented system
    Opts.param.tFinal = Opts.cora.timeStep;
    Opts.param.U = W_;
    Opts.param.R0 = zonotope(zeros(Opts.nxPlusnu, 1));

    % compute linear system + all necessary matrices
    [Opts.linearSystem, Opts.taylor] = ...
                            computeLinearSystemAndTaylorTerms(Opts,A_,D_);
end

function [A,B,D,nx,nu] = computeLinearSystemMatrices(benchmark,Opts)
% extract the system matrices for the linear model

    % compute dynamics
    [dynamics,nx,nu,nw] = computeDynamics(benchmark);

    % normal, i.e., non-augmented, system + controllable input matrices
    [modelIsLinear,A,B,D,c] = isLinearModel(dynamics,nx,nu,nw);

    % throw error if chosen system model is not linear or system type is 
    % not supported
    if ~modelIsLinear
        error(['The benchmark "', benchmark, '" is not a linear model!']);
    elseif any(abs(A*Opts.xEq + B*Opts.uEq + c) > 1e-10)
        error('Specified Opts.xEq and Opts.uEq is not a equilibrium point!');
    end
end

function K = computeStateFeedbackMatrix(Opts,A,B)
% compute feedback matrix 

    if isfield(Opts, 'K')
        
        K = Opts.K;
        
    else
        % construct discrete time system
        sys = linearSys(A,B);
        sysDT = linearSysDT(sys, Opts.cora.timeStep);

        % solve LQR problem
        K = -dlqr(sysDT.A,sysDT.B,Opts.Q,Opts.R);
    end
end

function [linSys,taylor] = computeLinearSystemAndTaylorTerms(Opts,A_,D_)
% compute auxiliary variables required for reachability analysis

    options = Opts.cora;
    params = Opts.param;
        
    % perform reachability to obtain linear system's property 'taylor'
    linSys = linearSys(A_,D_);
    options.linAlg = 'standard';

    reach(linSys,params,options);

    % simplify + save taylor terms to struct
    taylor.FCenter = center(linSys.taylor.F);
    taylor.FRadius = rad(linSys.taylor.F);
    taylor.eAt = linSys.taylor.eAt;
    taylor.Rpar = linSys.taylor.RV.deleteAligned();
    taylor.Rtrans = linSys.taylor.Rtrans.deleteAligned();
end

function [dynamics, nx, nu, nw] = computeDynamics(benchmark)
% get function handle to the dynamic function of benchmark model

    % get function handle
    str = ['dynamics = @(x,u,w)', benchmark, '(x,u,w);'];
    eval(str);

    % get number of states, inputs, and disturbances
    [count,out] = inputArgsLength(dynamics, 3);

    % extract variables
    nx = out;
    nu = count(2);
    nw = count(3);
end

function nameSolver = bestAvailableSolver(Opts)
% determine the best available convex optimization solver

    foundMOSEK =   canSolverBeFound('mosekopt');
    foundGUROBI =  canSolverBeFound('gurobi');

    if foundMOSEK
        nameSolver = 'mosek';
    elseif foundGUROBI
        nameSolver = 'gurobi';
    elseif strcmp(Opts.costFun,'sum')
        nameSolver = 'linprog';
    else
        nameSolver = 'fmincon';
    end
end

function found = canSolverBeFound(nameSolver)
% check if the given solver is available

    % check if solver can be found or not
    s = exist(nameSolver, 'file');
    found = (s~=0) && (s~=7);
end
