function Opts = loadParamAndOpts(benchmark,Param,Opts)
% LOADPARAMANDOPTS - load parameters and options
%
% Syntax:
%       Opts = LOADPARAMANDOPTS(benchmark,Param,Opts)
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
%       -Opts:   augmented struct containing the algorithm setting
%
% See Also:
%       linSysMPC
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
% Authors:      Max Beier, Laura Lützow, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % get number of states, inputs, and disturbances
    [dynamics,nx,nu,nw] = computeDynamics(benchmark);

    % check benchmark parameters
    Param = checkParam(Param,'linSysMPC',nx,nu,nw);

    % check algorithm settings
    Opts = checkOpts(Opts,'linSysMPC',Param,nx,nu);

    % augment algorithm options structure with additional new options
    Opts = augmentOpts(benchmark, Param, Opts);
    Opts.dynamics = dynamics;
end


% Auxiliary Functions -----------------------------------------------------

function Opts = augmentOpts(benchmark, Param, Opts)
% precompute values required for the algorithm and store them in the Opts
% struct

    % additional algorithm settings
    Opts.dt = Opts.tOpt/Opts.N;
    Opts.cora.timeStep = Opts.dt;

    % compute dynamics
    [A,B,D,c,nx,nu] = computeLinearSystemMatrices(benchmark);
    
    Opts.nx = nx;
    Opts.nu = nu;
    
    % construct augmented system matrices (states + input)
    B_ = [B; zeros(nu)];
    A_ = [[A; zeros(nu, nx)], B_];
    D_ = blkdiag(eye(nx),zeros(nu));
    c_ = [c; zeros(nu,1)];
    
    % compute disturbance set for augmented system
    W = zonotope(D * Param.W);
    W_ = zonotope([[W.c W.G]; zeros(nu,size([W.c W.G], 2))]);

    % compute feedback matrix [X; K*X] for augmented system
    K = computeStateFeedbackMatrix(Opts, A, B);
    Opts.K = K;
    Opts.XKX = [[eye(nx); K], zeros(Opts.nx + Opts.nu, nu)];
    
    % adapt parameter for reachability analysis to augmented system
    Opts.param.tFinal = Opts.cora.timeStep;
    Opts.param.U = W_;
    Opts.param.R0 = zonotope(zeros(Opts.nx + Opts.nu, 1));

    % compute linear system + all necessary matrices
    [Opts.linearSystem, Opts.taylor] = ...
                          computeLinearSystemAndTaylorTerms(Opts,A_,D_,c_);

    % copy parameter to options
    Opts = params2options(Param,Opts);

    % measurement error
    if isfield(Param,'V') && ~representsa(Param.V,'emptySet')
       Opts.V = cartProd(zonotope(Param.V), zeros(nu,1));
    else
       Opts.V = zonotope(zeros(nx+nu,1));
    end

    % replace goal state if it is on the boundary
    if any(abs(Opts.termReg.P.A * Opts.xf - Opts.termReg.P.b) < 1e-5)
        Opts.xf = center(Opts.termReg);
    end
end

function [A,B,D,c,nx,nu] = computeLinearSystemMatrices(benchmark)
% extract the system matrices for the linear model

    % compute dynamics
    [dynamics,nx,nu,nw] = computeDynamics(benchmark);

    % normal, i.e., non-augmented, system + controllable input matrices
    [modelIsLinear,A,B,D,c] = isLinearModel(dynamics,nx,nu,nw);

    % throw error if chosen system model is not linear
    if ~modelIsLinear
        error(['The benchmark "', benchmark, '" is not a linear model!']);
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

function [linSys,taylor] = computeLinearSystemAndTaylorTerms(Opts,A_,D_,c_)
% compute auxiliary variables required for reachability analysis

    options = Opts.cora;
    params = Opts.param;
        
    % perform reachability to obtain linear system's property 'taylor'
    linSys = linearSys(A_,D_,c_);
    options.linAlg = 'standard';

    options4taylor = options;
    params4taylor = params;

    reach(linSys,params,options);
    
    %%taylor term
    taylor_term = computetaylorterm(linSys, params.R0, params4taylor, options4taylor);


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

function taylor_term = computetaylorterm(obj,Rinit,params,options)

%%
options = validateOptionslinsysMPC(obj, 'reach',params,options);
    specLogic = [];
    if ~isempty(spec)
        [spec,specLogic] = splitLogic(spec);
        if ~isempty(spec)
    	    options.specification = spec;
        end
    end

%%
%% factors computation
for i=1:(options.taylorTerms+1)
    %compute initial state factor
    options.factor(i) = options.timeStep^(i)/factorial(i);    
end
%%

% compute exponential matrix
obj = exponential(obj,options);
% compute time interval error (tie)
obj = tie(obj,options);
% compute reachable set due to input
obj = inputSolution(obj,options);
%change the time step
obj.taylor.timeStep=options.timeStep;

%compute reachable set of first time interval
eAt=expm(obj.A*options.timeStep);
%save data to object structure
obj.taylor.eAt=eAt;

F=obj.taylor.F;
RV=obj.taylor.RV;
inputCorr=obj.taylor.inputCorr;
if iscell(obj.taylor.Rtrans)
    Rtrans=obj.taylor.Rtrans{1};
else
    Rtrans=obj.taylor.Rtrans;
end


%first time step homogeneous solution
Rhom_tp=eAt*Rinit + Rtrans;
if isa(Rinit,'polyZonotope') || isa(Rinit,'conPolyZono')
    Rhom=enclose(Rinit,Rhom_tp)+F*zonotope(Rinit)+inputCorr;
elseif isa(Rinit,'zonoBundle') 
    Rhom=enclose(Rinit,Rhom_tp)+F*Rinit.Z{1}+inputCorr;
else
    Rhom=enclose(Rinit,Rhom_tp)+F*Rinit+inputCorr;
end

%reduce zonotope
Rhom=reduce(Rhom,options.reductionTechnique,options.zonotopeOrder);
Rhom_tp=reduce(Rhom_tp,options.reductionTechnique,options.zonotopeOrder);
RV=reduce(RV,options.reductionTechnique,options.zonotopeOrder);

%save homogeneous and particulate solution
options.Rhom=Rhom;
options.Rhom_tp=Rhom_tp;
options.Raux=RV;
if strcmp(options.linAlg,'wrapping-free')
    options.Rpar=interval(RV);
else
    options.Rpar=RV;
end
options.Rtrans=obj.taylor.Rtrans;

%total solution
if isa(Rinit,'mptPolytope')
    %convert zonotopes to polytopes
    Radd=mptPolytope(RV);
    Rtotal=Rhom+Radd;
    Rtotal_tp=Rhom_tp+Radd;
else
    %original computation
    Rtotal=Rhom+RV;
    Rtotal_tp=Rhom_tp+RV;
end

%write results to reachable set struct Rfirst
Rfirst.tp=Rtotal_tp;
Rfirst.ti=Rtotal;
end