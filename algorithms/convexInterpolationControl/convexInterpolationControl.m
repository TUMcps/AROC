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
%           -.W:            set of uncertain disturbances (class: interval)
%           -.X:            set of state constraints (class: mptPolytope)
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
%           -.parallel:     use parallel computing
%                           [{false} / true]
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
%           -.extHorizon.decay:     decay function for the objective
%                                   function of the optimization problem
%                                   with extended optimization horizon
%                                   [{'fall+End'} / 'uniform' / 'fall' / 
%                                    'fallLinear' / 'fallLinear+End' / 
%                                    'fallEqDiff' / 'fallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
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
parallelo{1} = zonotope(Opts.R0);       % initial zonotope / parallelotope

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
for tc = 1:Opts.N 
    
    % compute extreme points of the parallelotope 
    % (see Line 5 of Alg. 1 in [1])
    v(:,:,tc) = symExtremePoints(parallelo{tc},Opts.I);       
    
    % select optimization horizon
    if Opts.extHorizon.active
        if ischar(Opts.extHorizon.horizon) && strcmp(Opts.extHorizon.horizon,'all')
            horizon = Opts.N-tc+1;
        else
            horizon = min(Opts.N-tc+1,Opts.extHorizon.horizon);
        end
    else
        horizon = 1;
    end  
    
    % solve local optimal control problems for all vertices 
    % (see Lines 6-8 of Alg. 1 in [1])
    [uCornerMultiExt,~] = localCornerControl(sys,xCenter(:,tc+1:tc+horizon), ...
                                v(:,:,tc),Opts.hinter,Opts.Q,Opts.R, ...
                                Opts.Ninter,horizon,Opts);
    
    % only the first few control values are applied to the system for the
    % case that a extendet optimization horizon is used
    uCornerMulti = uCornerMultiExt(:,:,1:Opts.Ninter);
    
    % display current iteration
    fprintf('Time step: %i\n',tc);
        
    % compute the controller and the corresponding reachable sets
    % (see Line 9 of Alg. 1 in [1])
    Opts.tStart = (tc-1)*Opts.hc;
    
    if strcmp(Opts.controller,'linear')
        [zonoExt{tc+1},Rtemp,controlLawParam{tc}] = ...
                            computeLinearController(dynamic,zono{tc}, ...
                            parallelo{tc},uCornerMulti,uCenter(:,tc),Opts);
    elseif strcmp(Opts.controller,'quadratic')   
        [zonoExt{tc+1},Rtemp,controlLawParam{tc}] = ...
                            computeQuadraticController(dynamic,zono{tc}, ...
                            parallelo{tc},uCornerMulti,Opts);
    else   
        [zonoExt{tc+1},Rtemp,controlLawParam{tc}] = ...
                            computeExactController(dynamic,zono{tc}, ...
                            parallelo{tc},uCornerMulti,Opts);
    end
    
    reachSet = add(reachSet,Rtemp);

    % save only the state part without the auxiliary states of the 
    % exteneded zonotope
    zono{tc+1} = project(zonoExt{tc+1},1:Opts.nx);

    if tc <= Opts.N
        
        % restructure the polynomial zonotope to increase dependent part
        if isa(zono{tc+1},'polyZonotope')
            if mod(tc,Opts.polyZono.N) == 0
                zono{tc+1} = polyZonotope(reduce(zonotope(zono{tc+1}),'pca',1));
                zonoTemp = zonotope(zono{tc+1});
            else
                zono{tc+1} = restructure(zono{tc+1},'reduceGirard', ...
                                         Opts.polyZono.orderDep, ...
                                         Opts.polyZono.order);
                                     
                zonoTemp = zonotope(zono{tc+1});
            end
        else
            zonoTemp = zono{tc+1};
        end

        % compute the parallelotope over-approximation of linear zonotope
        % (see Line 4 of Alg. 1 in [1])
        parallelo{tc+1} = reduce(zonoTemp,'pca',1);
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
    Opts.hinter = Opts.tFinal/(Opts.N*Opts.Ninter);
    Opts.ReachOpts.timeStep = Opts.hinter / Opts.reachSteps;
    Opts.useAcado = Opts.CentOpts.useAcado;
    Opts.uMax = Opts.CentOpts.uMax;
    Opts.uMin = Opts.CentOpts.uMin;
    Opts.hc = Opts.tFinal/Opts.N;
    
    Opts.CentOpts.Nc = Opts.N;               % number ref. traj. time steps
    Opts.CentOpts.hc = Opts.hc;              % size center traj. time steps
    
    if strcmp(Opts.controller,'exact')
       Opts.ReachOpts.lagrangeRem.simplify = 'collect'; 
    end
    
    % initialize dynamic files
    name = ['AROCconvInter',benchmark,Opts.controller, ...
            Opts.ReachOpts.alg,num2str(Opts.ReachOpts.tensorOrder)];
        
    if strcmp(Opts.controller,'linear')
       nExtended = 2*Opts.nx+Opts.nu;
       convInterContrLin = @(x,w) dynamicsClosedLoopLinear(x,w,Opts.nx,Opts.nu,Opts.funHandle);
       sys = nonlinearSys(name,@(x,w) convInterContrLin(x,w),nExtended,Opts.nw);
    elseif strcmp(Opts.controller,'quadratic')
       nExtended = 2*Opts.nx;            
       nParam = Opts.nu*(2*Opts.nx + 1); 
       convInterContrQuad = @(x,w,p) dynamicsClosedLoopQuadratic(x,w,p,Opts.nx,Opts.funHandle);
       sys = nonlinParamSys(name,@(x,w,p)convInterContrQuad(x,w,p),nExtended,Opts.nw,nParam);
    else
       nExtended = 2*Opts.nx;            
       nParam = 2^(Opts.nx)*(Opts.nu); 
       convInterContrExact = @(x,w,p) dynamicsClosedLoopExact(x,w,p,Opts.nx,Opts.funHandle);
       sys = nonlinParamSys(name,@(x,w,p)convInterContrExact(x,w,p),nExtended,Opts.nw,nParam);
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