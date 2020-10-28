function [objContr,res] = optimizationBasedControl(benchmark,Param,varargin)
% OPTIMIZATIONBASEDCONTROL - Implementation of a optimazation based control
%                            algorithm
%
% Syntax:
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param)
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param,Opts)
%       [objContr,res] = OPTIMIZATIONBASEDCONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for a control algorithm that is based on
%       the optimization over reachable sets
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
%           -.N:                number of time steps
%                               [{5} / positive integer]
%           -.reachSteps:       number of reachability steps during one
%                               time step (optimization)
%                               [{10} / positive integer]
%           -.reachStepsFin:    number of reachability steps during one
%                               time step (final reachable set computation)
%                               [{100} / positive integer]
%           -.maxIter:          maximum number of iterations for
%                               optimization with fmincon 
%                               [{15} / positive integer]
%           -.bound:            scaling factor between upper and lower
%                               bound of the weigthing matrices
%                               [{1000} / positive scalar]
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
%           -.cora.alg:                 reachability algorithm (nonlinear)
%                                       [{'lin'} / 'poly']
%           -.cora.linAlg:              reachability algorithm (linear)
%                                       [{'standard'} / 'fromStart' / 
%                                        'wrapping-free' / 'adap']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function
%                                       [{2} / 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{50} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{30} / positive integer]
%           -.cora.error:               uppper-bound for Hausdorff distance
%                                       (for cora.linAlg = 'adap' only)
%
%       -Post:      function handle to the postprocessing function that is
%                   used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objOptBasedContr)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objOptBasedContr
%
% References:
%       * *[1] Schuermann et al. (2017)*, Optimal Control of Sets of 
%              Solutions to Formally Guarantee Constraints of Disturbed 
%              Linear Systems
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
% Authors:      Ivan Hernandez, Niklas Kochdumper
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

Opts = initialization(benchmark,Param,Opts);   
     
     
%% Center Trajectory

sys = Opts.funHandle;

[xCenter,uCenter] = centerTrajectory(sys,Opts.CentOpts);
    
Opts.xc = xCenter;
Opts.uc = uCenter;


%% Algorithm

% initial value for optimiaztion variables
xInit = ones((Opts.nx+Opts.nu-1),Opts.N);

% linearize the system
if ~Opts.isLin 
    Opts = linearizeSystem(Opts);
    initializeReach(Opts);
end

% objective and constraint function for optimization
fun = @(x) costFun(x,Opts);
con = @(x) conFun(x,Opts); 

% upper and lower bounds for optimization variables
lb = 1/Opts.bound*ones(size(xInit));
ub = Opts.bound*ones(size(xInit));

% optimize the control law parameters using fmincon (see Eq. (8) in [1])
options = optimoptions('fmincon','MaxIter',Opts.maxIter,'Display','iter',...
                       'Algorithm','active-set','UseParallel',true, ...
                       'MaxFunEval',100000);
                   
xOpt = fmincon(fun,xInit,[],[],[],[],lb,ub,con,options);

% compute the reachable set for the optimized feedback matrix
[res,Rcont,R,K] = computeFinalReachSet(xOpt,Opts);

% check if constraints are satisfied
if ~res
   error('Failed to find a feasible solution!'); 
end


%% Assign output values

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{2};
   occSet = compOccupancySet(Rcont,post);
end

% bring parameters to the correct structure
if Opts.isLin
    dyn = Opts.linModel;
else
    dyn = Opts.funHandle;
end

contrLaw.K = K;
contrLaw.u_ref = uCenter;
contrLaw.N = Opts.N;

% construct controller object
objContr = objOptBasedContr(dyn,R{end},contrLaw,Param,occSet);  

% construct results object
res = results(Rcont,R,Opts.xc);

end


% Auxiliary Functions -----------------------------------------------------

function Opts = initialization(benchmark,Param,Opts)

    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);
    
    % initialize algorithm dependent settings
    Opts.uMin = infimum(Param.U);
    Opts.uMax = supremum(Param.U);
    Opts.x0 = center(Opts.R0);
    
    if isfield(Opts,'linModel')
       Opts.isLin = 1;
    else
       Opts.isLin = 0;
    end

    Opts.ReachParams.tFinal = Opts.tFinal/Opts.N;
    if ~Opts.isLin || ~strcmp(Opts.ReachOpts.linAlg,'adap')
        Opts.ReachOpts.timeStep = (Opts.tFinal/ Opts.N)/ Opts.reachSteps; 
    end
    
    Opts.CentOpts.Nc = Opts.N;               % number ref. traj. time steps
    Opts.CentOpts.hc = Opts.tFinal/Opts.N;   % size center traj. time step

    % store matrices for inequality constraints for input and state con
    if isfield(Param,'X')
       Opts.stateCon.A = get(Param.X,'A');
       Opts.stateCon.b = get(Param.X,'b');
    else
       Opts.stateCon = []; 
    end
    
    Opts.inputCon.A = [eye(Opts.nu);-eye(Opts.nu)];
    Opts.inputCon.b = [Opts.uMax;-Opts.uMin];
    
    % store model parameters
    if ~Opts.isLin
        % create nonlinear parameter system object for the closed loop
        name = ['AROCoptBased',benchmark,Opts.ReachOpts.alg, ...
                num2str(Opts.ReachOpts.tensorOrder)];
            
        funHan = @(x,u,p) closedLoopSystemNonlin(x,u,p,...
                                  Opts.funHandle,Opts.nx,Opts.nu,Opts.nw);
        
        Opts.sys = nonlinParamSys(name,@(x,u,p) funHan(x,u,p),2*Opts.nx, ...
                                  Opts.nw,(Opts.nx+1)*Opts.nu);
    end
end

function Opts = linearizeSystem(Opts)
% compute linearize models for each time step (required to obtain initial
% feedback matrices with LQR control)

    % calculate the jacobian matrix symbolically
    x = sym('x',[Opts.nx,1]);
    u = sym('u',[Opts.nu,1]);
    w = sym('w',[Opts.nw,1]);

    model = Opts.funHandle(x,u,w);

    Jx = jacobian(model,x);
    Ju = jacobian(model,u);

    Afun = matlabFunction(Jx,'Vars',{x,u,w});
    Bfun = matlabFunction(Ju,'Vars',{x,u,w});
    
    Opts.linModel.A = cell(Opts.N,1);
    Opts.linModel.B = cell(Opts.N,1);

    % loop over all center trajectory time steps
    for i = 1:Opts.N

        % linearize the system in the middle of the time step
        x_ = 0.5*(Opts.xc(:,i) + Opts.xc(:,i+1));
        u_ = Opts.uc(:,1);
        w_ = zeros(Opts.nw,1);

        Opts.linModel.A{i} = Afun(x_,u_,w_);
        Opts.linModel.B{i} = Bfun(x_,u_,w_);
    end
end

function initializeReach(Opts)
% exectue reachability analysis once to generate the files for tensors in
% CORA

    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    params.R0 = cartProd(Opts.R0,zonotope(Opts.x0));
    params.paramInt = zeros((Opts.nx+1)*Opts.nu,1);
    
    R = reach(Opts.sys,params,options);
end