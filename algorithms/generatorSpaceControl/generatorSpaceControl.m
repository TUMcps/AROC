function [objContr,res] = generatorSpaceControl(benchmark,Param,varargin)
% GENERATORSPACECONTROL - Implementation of a controller synthesis
%                         algorithm based on optimal control in generator 
%                         space
%
% Syntax:
%       [objContr,res] = GENERATORSPACECONTROL(benchmark,Param)
%       [objContr,res] = GENERATORSPACECONTROL(benchmark,Param,Opts)
%       [objContr,res] = GENERATORSPACECONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for the controller synthesis algorithm
%       that is based on optimal control in generator space.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/...")
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
%           -.N:            number of time steps
%                           [{10} / positive integer]
%           -.Ninter:       number of intermediate time steps during one
%                           time step
%                           [{4} / positive integer]
%           -.reachSteps:   number of reachability steps in one time step
%                           [{10} / positive integer]
%           -.Q:            state weighting matrix for the cost function of
%                           the optimal control problem
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the cost function of
%                           the optimal control problem
%                           [{zeros(nu)} / positive-definite square matrix]
%           -.refInput:     use the input from the reference trajectory
%                           as input for the center instead of optimizing
%                           [{false} / boolean]
%
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in center trajectory time steps
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
%                           (dimension: [nx,Opts.N*Opts.Ninter + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Opts.N*Opts.Ninter])
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       [{'lin'} / 'poly']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function [{2}/ 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{20} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{30} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{20} / positive integer]
%
%       -Post:      function handle to the postprocessing function that is
%                   used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objContrGenSpace)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objContrGenSpace
%
% References:
%       * *[1] Schuermann et al. (2017)*, Guaranteeing constraints of 
%              disturbed nonlinear systems using set-based optimal 
%              control in generator space
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
% Authors:      Jan Wagener, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


%% Initialization

Opts = [];
if nargin > 2
    Opts = varargin{1};    
end

[Opts,sys] = initialization(benchmark,Param,Opts);

% factors alpha that represent the control law
alpha = cell(Opts.N,1);

% parallelotope enclosure of the time point reachable set
P = cell(Opts.N+1,1);
P{1} = zonotope(Opts.R0);

% time point reachable set
R = cell(Opts.N+1,1);
R{1} = Opts.R0;

% time interval reachable set
reachSet = [];
     

%% Reference Trajectory

% compute reference trajectory (see Step 1 of Sec. 3.3 in [1]) 
dyn = Opts.funHandle;

[xCenter, uCenter] = centerTrajectory(dyn,Opts.CentOpts);
    
Opts.xc = xCenter;
Opts.uc = uCenter;


%% Algorithm

% compute linearized system dynamics (see Step 1 of Sec. 3.3 in [1]) 
[A,B,c,xf] = linearizeSystem(xCenter,uCenter,Opts);

% loop over all time steps
for i = 1:Opts.N
    
    fprintf('Time step: %i\n',i);
    Opts.tStart = (i-1)*Opts.dt*Opts.Ninter;
    
    % compute the control law (see Step 2 of Sec. 3.3 in [1])
    alpha{i} = computeAlpha(P{i},A,B,c,xf,i,Opts);
    
    % compute the reachable set (see Step 3 of Sec. 3.3 in [1])
    [Rtemp,R{i+1}] = reachSetGenSpaceContr(sys,R{i},P{i},alpha{i},Opts);
    
    reachSet = add(reachSet,Rtemp);
    
    % compute parallelotope enclosure of the reachable set
    P{i+1} = reduce(zonotope(R{i+1}),'pca',1);    
end


%% Assign output values

% remove the extendet states from the reachable set
reachSet = projectReachSet(reachSet,1:Opts.nx);

% check if the state constraints are satisfied
if isfield(Param,'X')
    checkStateConstraints(reachSet,Param.X);
end

% create results object
res = results(reachSet,R,xCenter);

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{2};
   occSet = compOccupancySet(reachSet,post);
end

% create controller object
contrLaw.alpha = alpha;
contrLaw.N = Opts.N;
contrLaw.Ninter = Opts.Ninter;
contrLaw.P = P;

objContr = objGenSpaceContr(Opts.funHandle,R{end},contrLaw,Param,occSet);

end


% Auxiliary Functions -----------------------------------------------------

function [Opts,sys] = initialization(benchmark,Param,Opts)

    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);

    % initialize algorithm dependent settings
    Opts.dt = Opts.tFinal/(Opts.N*Opts.Ninter);
    Opts.ReachOpts.timeStep = Opts.dt / Opts.reachSteps;
    
    Opts.CentOpts.Nc = Opts.N*Opts.Ninter;   % number ref. traj. time steps
    Opts.CentOpts.hc = Opts.dt;              % size center traj. time steps
    
    % generate function handles for linearized dynamics
    x = sym('x',[Opts.nx,1]); 
    u = sym('u',[Opts.nu,1]);
    w = sym(zeros(Opts.nw,1));
    
    f = Opts.funHandle(x,u,w);
    
    A = jacobian(f,x);
    B = jacobian(f,u);
    
    Opts.linDyn.A = matlabFunction(A,'Vars',{x,u});
    Opts.linDyn.B = matlabFunction(B,'Vars',{x,u});
    
    % create nonlinearSys object for the closed-loop dynamics
    nExtended = 2*Opts.nx + Opts.nu;
    name = ['AROCgenSpace',benchmark,Opts.ReachOpts.alg, ...
            num2str(Opts.ReachOpts.tensorOrder)];
    
    funHan = @(x,w) dynamicsClosedLoopLinear(x,w,Opts.nx, ...
                                               Opts.nu,Opts.funHandle);
                                           
    sys = nonlinearSys(name,@(x,w) funHan(x,w),nExtended,Opts.nw);
end