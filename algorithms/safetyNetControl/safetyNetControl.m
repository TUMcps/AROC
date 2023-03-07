function [objContr,res] = safetyNetControl(benchmark,Param,Opts,varargin)
% SAFETYNETCONTROL - Implementation of the safety net controller based on
%                    backward rechable set computation
%
% Syntax:
%       [objContr,res] = SAFETYNETCONTROL(benchmark,Param,Opts)
%       [objContr,res] = SAFETYNETCONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for the Safety Net Control Algorithm 
%       Algorithm. 
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
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.N:            number of time-steps
%                           [{10} / positive integer]
%           -.Ninter:       number of intermediate time steps
%                           [{4} / positive integer]
%           -.reachSteps:   number of reachability steps in one time step
%                           [{10} / positive integer]
%           -.order:        maximum zonotope order of backward reach. sets
%                           [{3} / positive integer]
%           -.iter:         number of iterations for optimization
%                           [{1} / positive integer]
%           -.realTime:     flag specifying if real time computation time 
%                           constraints are considered (Opts.realTime = 1) 
%                           or not (Opts.realTime = 0) [{false} / boolean]
%           -.tComp:        time allocated to perform the computations for 
%                           the optimizations (0 < tComp < tFinal/N/Ninter)
%           -.controller:   string specifying the comfort controller that 
%                           is applied during online execution
%           -.contrOpts:    struct with confort controller settings
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
%       -Post:      function handle to the postprocessing function that is
%                   used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objSafteyNetContr)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objSafetyNetContr
%
% References:
%       * *[1] Schuermann et al. (2021)*, Formal Safety Net Control Using
%              Backward Reachability Analysis
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


%% Initialization

[sys,Opts] = initialization(benchmark,Param,Opts);

% cell-array storing the time point reach. set at each ref. traj. time step
zono = cell(Opts.N+1,1);
zono{end} = zonotope(Opts.Xf);
R0 = zono{end};

% cell-array storing the time interval reachable set
Rcont = cell(Opts.N,1);

% matrix storing the generator-to-input assignments
H = cell(Opts.N,1);


%% Reference Trajectory

% compute center trajectory
dyn = Opts.funHandle;
[xCenter,uCenter] = centerTrajectory(dyn,Opts.CentOpts);
    
% compute the adapted input sets shifted by the center trajectory
[U,xCenter,uCenter] = subsetsInput(xCenter,uCenter,Opts);


%% Algorithm 

% loop backward over all center-trajectory time steps
for i = Opts.N:-1:1
    
    fprintf('Iteration: %i\n\n',Opts.N-i+1);
    Opts.tStart = (i-1)*Opts.timeStep;
    
    % backward reachable set for linearized dynamics (Sec. V.C. in [1])
    [Rtemp,H{i}] = linearBackwardReach(R0,xCenter{i},uCenter{i},U{i},Opts);
    
    % scale the initial set to obtain the backward reachable set for the
    % nonlinear dynamics (Sec. V.E in [1])
    [zono{i},Rcont{i}] = scaleInitialSet(sys,Rtemp,H{i},zono{i+1},U{i},...
                                         i,Opts);
                                         
    % reduce the order of the zonotope
    R0 = orderReduction(zono{i},Opts.order);
end


%% Assign output values

% construct reachSet object
reachSet = Rcont{1};
for i = 2:length(Rcont)
   reachSet = add(reachSet,Rcont{i}); 
end
reachSet = projectReachSet(reachSet,1:Opts.nx);

% check if the state constraints are satisfied
if isfield(Param,'X')
    checkStateConstraints(reachSet,Param.X);
end

% construct comfort controller
Opts.xCenter = xCenter;
Opts.uCenter = uCenter;
listComf = cell(length(Opts.controller),1);

for i = 1:length(Opts.controller)
    str = ['objComf = comfContr',Opts.controller{i},'(''',benchmark, ...
            ''',Opts,Opts.contrOpts{i});'];
    eval(str);
    listComf{i} = objComf;
end

% create results object
res = results(reachSet,zono,xCenter,[]);

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{1};
   occSet = compOccupancySet(reachSet,post);
end

% create controller object
contrLaw.comfContr = listComf;
contrLaw.reachSet = zono;
contrLaw.H = H;
contrLaw.N = Opts.N;
contrLaw.Ninter = Opts.Ninter;
contrLaw.Rinit = Param.R0;
contrLaw.Usub = U;
contrLaw.tComp = Opts.tComp;
contrLaw.realTime = Opts.realTime;

if isempty(Opts.V)    
    Param.R0 = zono{1};
else
    Param.R0 = minus(mptPolytope(zono{1}),Opts.V,'approx'); 
end

objContr = objSafetyNetContr(Opts.funHandle,zono{end},contrLaw,Param,occSet);

end


% Auxiliary Functions -----------------------------------------------------

function [sys,Opts] = initialization(benchmark,Param,Opts)
    
    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);
    
    % reference trajectory settings    
    Opts.CentOpts.Nc = Opts.N*Opts.Ninter;
    Opts.CentOpts.hc = Opts.tFinal/(Opts.N*Opts.Ninter);
    
    Opts.U = Param.U;
    Opts.uMinTotal = infimum(Opts.U);
    Opts.uMaxTotal = supremum(Opts.U);
    
    % compute goal set
    Opts.Xf = Param.R0 - center(Param.R0) + Param.xf;

    % time steps sizes
    Opts.timeStep = Opts.tFinal/Opts.N;
    Opts.dt = Opts.tFinal/(Opts.N*Opts.Ninter);
    
    Opts.ReachOpts.timeStep = Opts.dt/Opts.reachSteps;
    
    % store matrices for inequality constraints for input and state con
    if isfield(Param,'X')
       Opts.stateCon.A = get(Param.X,'A');
       Opts.stateCon.b = get(Param.X,'b');
    else
       Opts.stateCon = []; 
    end
    
    % final and initial state
    Opts.xf = center(Opts.Xf);
    Opts.x0 = center(Param.R0);
    
    % number of generators
    Opts.nGen = Opts.order * Opts.nx + Opts.nu*Opts.Ninter;
    
    % convert control law parameters to cell
    if ~iscell(Opts.controller)
       Opts.controller = {Opts.controller};
       Opts.contrOpts = {Opts.contrOpts};
    end
    
    % function handle to dynamic file
    str = ['Opts.funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % generate function handles for linearized dynamics
    x = sym('x',[Opts.nx,1]); 
    u = sym('u',[Opts.nu,1]);
    w = sym(zeros(Opts.nw,1));
    
    f = Opts.funHandle(x,u,w);
    
    A = jacobian(f,x);
    B = jacobian(f,u);
    
    Opts.linDyn.A = matlabFunction(A,'Vars',{x,u});
    Opts.linDyn.B = matlabFunction(B,'Vars',{x,u});
    
    % generate system dynamics
    nExtended = Opts.nx + Opts.nu + Opts.nGen;
    name = ['AROCsafetyNet',benchmark];
    
    funHan = @(x,w) dynamicsClosedLoopLinear(x,w,Opts.nx, ...
                                               Opts.nu,Opts.funHandle);
                                           
    sys = nonlinearSys(name,@(x,w) funHan(x,w),nExtended,Opts.nw);
end