function [objContr,res] = convexInterpolationControl(benchmark,Param,varargin)
% CONVEXINTERPOLATIONCONTROL - Implementation of the convex
%                              interpolation control algorithm in [1]
%
% Syntax:
%       objContr = CONVEXINTERPOLATIONCONTROL(benchmark,Param)
%       objContr = CONVEXINTERPOLATIONCONTROL(benchmark,Param,Opts)
%       objContr = CONVEXINTERPOLATIONCONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for the Convex Interpolation Control 
%       Algorithm. 
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -Param:             a structure containing the benchmark parameters
%
%           -.R0:           initial set of states (class: interval)
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal state should be
%                           reached
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval 
%                           or zonotope)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: polytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.controller:   use exact convex interpolation control law or a 
%                           linear or quadratic approximation
%                           [{'linear'} / 'quadratic' / 'exact'] 
%           -.N:            number of time-steps
%                           [{10} / positive integer]
%           -.Ninter:       number of intermediate time steps during one
%                           time step
%                           [{4} / positive integer]
%           -.reachSteps:   number of reachability steps in one time step
%                           [{20} / positive integer]
%           -.Q:            state weighting matrix for the cost function of
%                           the optimal control problem
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the cost function of
%                           the optimal control problem
%                           [{zeros(nu)} / positive-definite square matrix]
%
%           -.approx.method:        method used to determine the
%                                   approximated control law
%                                   [{'scaled'} / 'optimized' / 'center']
%           -.approx.lambda:        tradeoff betwen vertex inputs and
%                                   difference from the exact control law
%                                   [{0.5} / value between 0 and 1]
%
%           -.polyZono.N:           number of reference trajectory time
%                                   steps after which the polynomial
%                                   zontope is over-approximated with a
%                                   parallelotope
%                                   [{Opts.N/2} / positive integer]
%           -.polyZono.orderDep:    maximum zonotope order for the 
%                                   dependent part of the polynomial 
%                                   zonotope (for function restructure)
%                                   [{10} / positive integer]
%           -.polyZono.order:       maximum zonotope order for the
%                                   overall polynomial zonotope (for
%                                   function restructure)
%                                   [{20} / positive integer]
%
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in reference trajectory time 
%                                   steps
%                                   [{'all'} / positive integer]
%           -.extHorizon.weightingFunction:     instance of enumeration
%                                               class extendedHorizonWeights 
%                                               specifying the weighting 
%                                               function for the extended 
%                                               horizon 
%
%           -.refTraj.Q:    state weighting matrix for the cost function of
%                           optimal control problem (dimension:[nx,nx])
%           -.refTraj.R:    input weighting matrix for the cost function of
%                           optimal control problem (dimension:[nu,nu])
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,Opts.N + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Opts.N])
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       ('lin' or 'poly')
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function (2 or 3)
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{20} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{100} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{50} / positive integer]
%
%       -Post:      function handle to the postprocessing function that is
%                   used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objConvInterContr)
%       -res:       results object storing the computed reachable set and
%                   the reference trajectory
%
% See Also:
%       objConvInterContr
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


%% Initialization

% parse input arguments and user settings
Opts = [];
if nargin > 2
    Opts = varargin{1};    
end

[Opts,dynamic] = initialization(benchmark,Param,Opts);

% define variables
v = zeros(Opts.nx,2^Opts.nx,Opts.N+1);  % parallelotope vertices
zonoExt = cell(Opts.N+1,1);             % extended time point reachable set
controlLawParam = cell(Opts.N,1);       % control law parameter
reachSet = [];                          % object storing the reachable set

parallelo = cell(Opts.N+1,1);           % parallelotope enclosures
if representsa(Opts.V,'emptySet')
    parallelo{1} = zonotope(Opts.R0);     
else
    parallelo{1} = reduce(zonotope(Opts.R0)+Opts.V,'pca',1); 
end

zono = cell(Opts.N+1,1);                % time point reachable set
zono{1} = Opts.R0;                      % initial set 

     
%% Reference Trajectory

% compute reference trajectory (see Line 1 of Alg. 1 in [1])
sys = Opts.funHandle;

[xCenter,uCenter] = centerTrajectory(sys,Opts.CentOpts);
    
Opts.xc = xCenter;
Opts.uc = uCenter;  
     

%% Algorithm

% convex interpolation control algorithm (see Alg. 1 in [1])
for kk = 1:Opts.N 
    
    % compute extreme points of the parallelotope 
    % (see Line 5 of Alg. 1 in [1])
    v(:,:,kk) = symExtremePoints(parallelo{kk},Opts.I);       
    
    % select (extended) optimization horizon M
    if Opts.extHorizon.active
        if ischar(Opts.extHorizon.horizon) && strcmp(Opts.extHorizon.horizon,'all')
            M = Opts.N-kk+1;
        else
            M = min(Opts.N-kk+1,Opts.extHorizon.horizon);
        end
    else
        M = 1;
    end  
    
    % solve local optimal control problems for all vertices 
    % (see Lines 6-8 of Alg. 1 in [1])
    [uCornerMultiExt,~] = localVertexControl( ...
        sys,v(:,:,kk),xCenter(:,kk+1:kk+M),M,Opts.lvcOpts);
    
    % only the first few control values are applied to the system for the
    % case that a extended optimization horizon is used
    uCornerMulti = uCornerMultiExt(:,:,1:Opts.Ninter);
    
    % display current iteration
    fprintf('Time step: %i\n',kk);
        
    % compute the controller and the corresponding reachable sets
    % (see Line 9 of Alg. 1 in [1])
    Opts.tStart = (kk-1)*Opts.dt;
    
    if strcmp(Opts.controller,'linear')
        [zonoExt{kk+1},Rtemp,controlLawParam{kk}] = ...
                            computeLinearController(dynamic,zono{kk}, ...
                            parallelo{kk},uCornerMulti,uCenter(:,kk),Opts);
    else   
        [zonoExt{kk+1},Rtemp,controlLawParam{kk}] = ...
                            computeExactController(dynamic,zono{kk}, ...
                            parallelo{kk},uCornerMulti,Opts);
    end
    
    reachSet = add(reachSet,Rtemp);

    % save only the state part without the auxiliary states of the 
    % exteneded zonotope
    zono{kk+1} = project(zonoExt{kk+1},1:Opts.nx);

    if kk <= Opts.N
        
        % restructure the polynomial zonotope to increase dependent part
        if isa(zono{kk+1},'polyZonotope')
            if mod(kk,Opts.polyZono.N) == 0
                zono{kk+1} = polyZonotope(reduce(zonotope(zono{kk+1}),'pca',1));
                zonoTemp = zonotope(zono{kk+1});
            else
                zono{kk+1} = restructure(zono{kk+1},'reduceGirard', ...
                                         Opts.polyZono.orderDep, ...
                                         Opts.polyZono.order);
                                     
                zonoTemp = zonotope(zono{kk+1});
            end
        else
            zonoTemp = zono{kk+1};
        end

        % compute the parallelotope over-approximation of linear zonotope
        % (see Line 4 of Alg. 1 in [1])
        if representsa(Opts.V,'emptySet')
            parallelo{kk+1} = reduce(zonoTemp,'pca',1);
        else
            parallelo{kk+1} = reduce(zonoTemp + Opts.V,'pca',1);
        end
    end
end


%% Assign output values

% remove the extendet states from the reachable set
reachSet = projectReachSet(reachSet,1:Opts.nx);

% check if the state constraints are satisfied
if isfield(Param,'X')
    checkStateConstraints(reachSet,Param.X);
end

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{2};
   occSet = compOccupancySet(reachSet,post);
end

% contruct controller object
contrLaw.controller = Opts.controller;
contrLaw.Param = controlLawParam;
contrLaw.Nc = Opts.N;
contrLaw.Ninter = Opts.Ninter;
contrLaw.P = parallelo;

objContr = objConvInterContr(Opts.funHandle,zono{end},contrLaw,Param,occSet);
               
% store reachable set in results object
res = results(reachSet,zono,xCenter);

end


% Auxiliary Functions -----------------------------------------------------

function [Opts,sys] = initialization(benchmark,Param,Opts)

    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);

    % initialize algorithm dependent settings
    dt_inter = Opts.tFinal/(Opts.N*Opts.Ninter);
    Opts.ReachOpts.timeStep = dt_inter / Opts.reachSteps;
    Opts.uMax = supremum(Param.U);
    Opts.uMin = infimum(Param.U);
    Opts.dt = Opts.tFinal/Opts.N;
    
    Opts.CentOpts.N = Opts.N;               % number ref. traj. time steps
    Opts.CentOpts.dt = Opts.dt;             % size center traj. time steps

    % options for computing the local vertex control inputs
    Opts.lvcOpts.nx = Opts.nx;
    Opts.lvcOpts.nu = Opts.nu;
    Opts.lvcOpts.nw = Opts.nw;
    Opts.lvcOpts.dt = Opts.CentOpts.dt;
    Opts.lvcOpts.Ninter = Opts.Ninter;
    Opts.lvcOpts.Q = Opts.Q;
    Opts.lvcOpts.R = Opts.R;
    Opts.lvcOpts.X = Opts.X;
    Opts.lvcOpts.U = Opts.U;
    Opts.lvcOpts.W = Opts.W;
    Opts.lvcOpts.extHorizonWeightingFunction = Opts.extHorizon.weightingFunction;
        
    if strcmp(Opts.controller,'exact')
       Opts.ReachOpts.lagrangeRem.simplify = 'collect'; 
    end
    
    if ~strcmp(Opts.controller,'linear') || ~representsa(Opts.V,'emptySet')
       Opts.ReachOpts.intermediateTerms = 4; 
    end
    
    % initialize dynamic files
    name = ['AROCconvInter',benchmark,Opts.controller, ...
            Opts.ReachOpts.alg,num2str(Opts.ReachOpts.tensorOrder)];
        
    if strcmp(Opts.controller,'linear')
        if representsa(Opts.V,'emptySet')
            nExt = 2*Opts.nx+Opts.nu;
            funHan = @(x,w) dynamicsClosedLoopLinear(x,w, ...
                                           Opts.nx,Opts.nu,Opts.funHandle);
            sys = nonlinearSys(name,@(x,w) funHan(x,w),nExt,Opts.nw);
        else
            nExt = 3*Opts.nx;
            funHan = @(x,w,p) dynamicsClosedLoopLinearMeasErr( ...
                                     x,w,p,Opts.nx,Opts.nu,Opts.funHandle);
            sys = nonlinParamSys(name,@(x,w,p) funHan(x,w,p), ...
                                 nExt,Opts.nw,Opts.nu*(Opts.nx+1));
        end
    elseif strcmp(Opts.controller,'exact')
        if representsa(Opts.V,'emptySet')
            nExt = 2*Opts.nx;            
            nParam = 2^(Opts.nx)*(Opts.nu); 
            funHan = @(x,w,p) dynamicsClosedLoopExact(x,w, ...
                                                p,Opts.nx,Opts.funHandle);
            sys = nonlinParamSys(name,@(x,w,p)funHan(x,w,p), ...
                                 nExt,Opts.nw,nParam);
        else
            nExt = 3*Opts.nx;            
            nParam = 2^(Opts.nx)*(Opts.nu); 
            funHan = @(x,w,p) dynamicsClosedLoopExactMeasErr( ...
                                             x,w,p,Opts.nx,Opts.funHandle);
            sys = nonlinParamSys(name,@(x,w,p)funHan(x,w,p), ...
                                 nExt,Opts.nw,nParam);
        end
    end 

    % set-up parallel computing
    v = ver;
    if Opts.parallel && any(strcmp('Parallel Computing Toolbox', {v.Name}))
       gcp;
    else
       Opts.parallel = 0;
    end

    % construct matrix I storing the alpha values for the vertices of a 
    % parallelotope
    I = [1 -1];
    for i = 1:Opts.nx-1
        I = [ones(1,2^i) -ones(1,2^i); I I];
    end
    Opts.I = I;
end