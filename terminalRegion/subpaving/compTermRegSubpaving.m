function T = compTermRegSubpaving(benchmark,Param,Opts)
% COMPTERMREGSUBPAVING - compute terminal region with subpaving algorithm
%
% Syntax:
%       T = COMPTERMREGSUBPAVING(benchmark,Param,Opts)
%
% Description:
%       This function first computes a terminal controller that stabilizes
%       the system around the specified equilibrium point using a Linear
%       Quadratic Regulator, and then computes a terminal region for the
%       contolled system with the subpaving algorithm in [1].
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -Param:             a structure containing the benchmark parameters
%
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval 
%                           or zonotope)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.Tdomain:      search domain for the terminal region (class:
%                           interval)
%           -.Tinit:        initial guess for the terminal region (class: 
%                           interval) 
%           -.xEq:          states for the equibrium point of the system
%           -.uEq:          control inputs for the equilibrium point of the
%                           system
%           -.K:            feedback matrix for the terminal contorller
%           -.Q:            state weighting matrix for the LQR approach
%                           applied to determine the terminal controller
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the LQR approach
%                           applied to determine the terminal controller
%                           [{zeros(nu)} / positive-definite square matrix]
%           -.numRef:       number of refinement steps for the box size
%                           [{4}, positive integer]
%           -.tMax:         final time for reachability analysis
%           -.reachSteps:   number of reachability steps in one time step
%                           [{20} / positive integer]
%           -.enlargeFac:   enlargement factor for enlargin the initial
%                           guess Opts.Tinit
%                           [{1.5}, double > 1]
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       ('lin' or 'poly')
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function (2 or 3)
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{50} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{50} / positive integer]
%
% Output Arguments:
%
%       -T:     object of class termRegSubpaving
%
% See Also:
%       terminalRegion, termRegSubpaving
%
% References:
%       * *[1] El-Guindy et al. (2017)*, Estimating the region of 
%              attraction via forward reachable sets, ACC 2017
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
% Authors:      Niklas Kochdumper, Ahmed El-Guindy
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % initialization
    Opts = initialization(benchmark,Param,Opts);
    
    
    % Control Law ---------------------------------------------------------
    
    % compute linearized system matrices
    x = sym('x',[Opts.nx,1]); 
    u = sym('u',[Opts.nu,1]);
    w = sym(zeros(Opts.nw,1));
    
    f = Opts.funHandle(x,u,w);
    
    Asym = jacobian(f,x);
    Bsym = jacobian(f,u);
    
    Afun = matlabFunction(Asym,'Vars',{x,u});
    Bfun = matlabFunction(Bsym,'Vars',{x,u});
    
    A = Afun(Opts.xEq,Opts.uEq);
    B = Bfun(Opts.xEq,Opts.uEq);
    
    % compute feedback matrix with a Linear Quadratic Regulator (LQR)
    if ~isfield(Opts, 'K')
        [K,~,~] = lqr(A,B,Opts.Q,Opts.R);
        K = -K;
    else
        K = Opts.K; 
    end
    
    % initialize search domain if it is not given
    if isempty(Opts.Tdomain)
        if isfield(Param,'X')
            Opts.Tdomain = initSearchDomain(K,Opts.xEq,Opts.uEq, ...
                                                        Param.U,Param.X);
        else
            Opts.Tdomain = initSearchDomain(K,Opts.xEq,Opts.uEq,Param.U);
        end
    end
    
    % initialize closed-loop system
    name = ['AROCsubpaving',benchmark];
    
    if isempty(Opts.V)
        dynFun = @(x,w) Opts.funHandle(x,Opts.uEq + K*(x-Opts.xEq),w);
        sys = nonlinearSys(name,dynFun,Opts.nx,Opts.nw);
    else
        dynFun = @(x,w) Opts.funHandle(x,Opts.uEq + K*(x + ...
                                w(Opts.nw+1:end) - Opts.xEq),w(1:Opts.nw));
        sys = nonlinearSys(name,dynFun,Opts.nx,Opts.nw + Opts.nx);
    end


    % Part 1: Enlarge around Equilibrium Point ----------------------------
    
    % initialize reachability analysis
    initializeReach(sys,Opts);
    
    % check if the initial set converges into itself
    D = Opts.Tinit;
    ver = compReachSet(sys,D,D,K,Opts);
    
    while ver
       
        % enlarge set
        D_ = enlarge(D,Opts.enlargeFac);
        
        % compute reachable set
        res = compReachSet(sys,D_,D,K,Opts);
        
        if ~res
            break; 
        else
            D = D_;
        end
    end
    
    
    % Part 2: Increase Terminal Region ------------------------------------

    list_out = {Opts.Tdomain};
    list_in = {};
    
    % loop over different sizes of boxes
    for i = 1:Opts.numRef
       
        list_out_ = {};
        
        % loop over all boxes in the list
        for j = 1:length(list_out)
            
            % divide the current box into subboxes
            P = partition(list_out{j},2);
            
            % loop over all partitions
            for k = 1:length(P)
               
                % compute reachables set starting form the current box
                res = compReachSet(sys,P{k,1},D,K,Opts);
                
                if res
                    list_in{end+1} = P{k,1};
                else
                    list_out_{end+1} = P{k,1};
                end
            end
        end
        
        % update list
        list_out = list_out_;
    end
    
    
    % Polytope Under-Approximation ----------------------------------------
    
    % determine suitable polytope halfspace directions
    C = polytopeDirections(list_in);
    
    % compute the tightest polytope enclosing the terminal region
    d = zeros(size(C,1));
    
    for i = 1:length(list_in)
       Z = zonotope(list_in{i}) - Opts.xEq; 
       temp = supremum(interval(C*Z));
       d = max([d,temp],[],2);
    end
    
    % scale polytope so that it is inner-approximation of terminal region
    poly = scalePolytope(C,d,list_out,D,Opts);
    
    % if the initial set Tinit did not converge into itself check if it is
    % fully enclosed by the polytope -> polytope is valid terminal region
    if ~ver
       
        if ~contains(poly,Opts.Tinit)
           
            % compute maximum contained box
            n = length(Opts.Tinit);
            C_ = [C(n+1:2*n,:); C(3*n+1:end,:)];
            d_ = [d(n+1:2*n); d(3*n+1:end)];
            poly = scalePolytope(C_,d_,list_out,D,Opts,diag(1./d_));
            
            if ~contains(poly,Opts.Tinit)
               error('Failed to compute a valid terminal region!') 
            end 
        end
    end
    
    % construct terminal region object
    T = termRegSubpaving(Opts.funHandle,poly,K,list_in, ...
                         Opts.xEq,Opts.uEq,Param);
end


% Auxiliary Functions -----------------------------------------------------

function Opts = initialization(benchmark,Param,Opts)

    % function handle to dynamic file
    str = ['funHan = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % get number of states, inputs, and disturbances
    [count,out] = inputArgsLength(funHan,3);
    
    nx = out;
    nu = count(2);
    nw = count(3);
    
    % check if the specified paramters are valid
    Param = checkParam(Param,'subpaving',nx,nu,nw);
    Opts = checkOpts(Opts,'subpaving',Param,nx,nu);

    % copy benchmark parameter to options
    Opts = params2options(Param,Opts);
    Opts.funHandle = funHan;
    Opts.nx = nx;
    Opts.nu = nu;
    Opts.nw = nw;
    
    % define settings for CORA toolbox
    if isempty(Opts.V)
        param.U = zonotope(Opts.W);
    else
        param.U = cartProd(zonotope(Opts.W),zonotope(Opts.V)); 
    end
    param.tFinal = Opts.tMax/Opts.reachSteps;
    
    Opts.timeStep = Opts.tMax/Opts.reachSteps;
    Opts.ReachParam = param;
    Opts.ReachOpts = Opts.cora;
    Opts.ReachOpts.timeStep = Opts.tMax/Opts.reachSteps;   
end

function initializeReach(sys,Opts)
% initialize reachability analysis (required to generate tensors for the
% system)

    % initialization
    options = Opts.ReachOpts;
    param = Opts.ReachParam;
    
    param.R0 = zonotope(Opts.Tinit);
    
    % reachability analysis
    reach(sys,param,options);
end

function res = compReachSet(sys,R0,target,K,Opts)
% compute the reachable set of the closed loop system

    % initialization
    options = Opts.ReachOpts;
    params = Opts.ReachParam;

    params.R0 = zonotope(R0);
    t = 0:Opts.timeStep:Opts.tMax;
    res = 0;

    % loop until final time is reached
    for i = 1:length(t)
        
       % compute reachable set 
       try
           R = reachNonlinear(sys,options,params);
       catch
           return;
       end
       
       % check if input constraints and state constraints are satisfied
       for j = 1:length(R.timeInterval.set)
          
           % check state constraints
           if isfield(Opts,'X') && ~contains(Opts.X,R.timeInterval.set{j})
               return;
           end
           
           % check input constraints
           if isempty(Opts.V)
               U = Opts.uEq + K*(R.timeInterval.set{j} + (-Opts.xEq)); 
           else
               U = Opts.uEq+K*(R.timeInterval.set{j}+Opts.V+(-Opts.xEq)); 
           end
           
           if ~contains(Opts.U,U)
              return;
           end
       end
       
       % check if reachable set is in target set
       if contains(target,R.timePoint.set{end})
          res = 1;
          return;
       end
       
       params.R0 = R.timePoint.set{end};
    end
end

function C = polytopeDirections(list)
% determine suitable directions for the polytope halfspace normal vectors
% using Principal Componten Analysis

    % compute vertices of all boxes that define the terminal region
    V = [];
    
    for i = 1:length(list)
       V = [V,vertices(list{i})]; 
    end

    % compute suitable directions using Principal Component Analysis
    [S,~,~] = svd(V);
    
    C1 = S';
    C2 = -S';

    % add unit vectors as additional directions
    C1 = [C1; eye(size(V,1))];
    C2 = [C2; -eye(size(V,1))];
    
    C = [C1;C2];
end

function poly = scalePolytope(C,d_,list,D,Opts,varargin)
% scale the polytope that is an outer approximation of the terminal region 
% to obtain an inner approximation of terminal region using linear
% programming

    % define optimization variables for YALMIP
    d = sdpvar(size(C,1),1);

    % define objective function and constraints
    if nargin > 5
        W = varargin{1};
        Objective = -sum(W*d);
    else
        Objective = -sum(d);
    end
    
    Constraints = [d >= 1/1000*min(rad(D))*ones(size(d)); ... 
                   d <= d_; ... 
                   C*zeros(size(Opts.xEq)) <= d; ...
                   d(1:size(C,1)/2) == d(size(C,1)/2+1:end)];

    % loop over all boxes
    for i = 1:length(list)

        v = zeros(size(C,1),1);
        
        % loop over all polytope halfspaces
        for j = 1:size(C,1)

            % convert box to zonotope and center at refrence point zero
            Z = zonotope(list{i}) - Opts.xEq;

            % compute maximum required hyperplane offset so that the box is 
            % located outside the halfspace
            c = center(Z);
            G = generators(Z);

            v(j) = C(j,:)*c - sum(abs(C(j,:)*G));
        end

        % each box has to be outside of at least one polytope halfspace
        Constraints = [Constraints; max(v-d) >= 0];
    end

    % solve linear program with the YALMIP toolbox
    optimize(Constraints,Objective,sdpsettings('verbose',0));

    % store the solution
    d = value(d);

    % define resulting polytope
    poly = mptPolytope(C,d) + Opts.xEq;
end