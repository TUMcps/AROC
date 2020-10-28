function res = reachsetMPC(benchmark,Param,Opts)
% REACHSETMPC - Implementation of the Reachset Model Predictive Control
%               algorithm
%
% Syntax:
%       res = REACHSETMPC(benchmark,Param,Opts)
%
% Description:
%       Implementation of the Reachset Model Predictive Control algorithm
%       as described in [1]
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -Param:             a structure containing the benchmark parameters
%
%           -.R0:           initial set (class: interval)
%           -.xf:           goal state
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.tOpt:         final time for the optimization  
%           -.N:            number of time steps
%                           [{10} / positive integer]
%           -.reachSteps:   number of reachability steps during one time 
%                           step [{10} / positive integer]
%           -.U_:           tightened set of admissible control inputs 
%                           (class: interval)
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope)
%           -.Q:            state weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.R:            input weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.Qlqr:         state weighting matrix for the cost function of
%                           the LQR approach (tracking controller)
%           -.Rlqr:         input weighting matrix for the cost function of
%                           the LQR approach (tracking controller)
%           -.realTime:     flag specifying if real time computation time 
%                           constraints are considered (Opts.realTime = 1) 
%                           or not (Opts.realTime = 0) [{true} / boolean]
%           -.tComp:        time allocated to perform the computations for 
%                           the optimizations (0 < tComp < tOpt/N).
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%           -.maxIter:      maximum number of iterations for the optimal
%                           control problem [{10} / positive integer]
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       [{'lin'} / 'poly']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function [{2}/ 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{5} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{3} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{3} / positive integer]
%
% Output Arguments:
%
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       convexInterpolationControl
%
% References:
%       * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
%              for Disturbed Nonlinear Systems
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

% initialize settings and generate dynamics of the controlled system
[Opts,sys] = initialization(benchmark,Param,Opts);     

% initialize costs
J = 1e6;
L = inf;

x0 = randPoint(Opts.R0);


%% Initial Solution

% compute an initial feasible solution (see Line 1 of Alg. 1 in [1])

% solve optimal control problem
[param,res] = optimalControlReachsetMPC(x0,J,L,Opts);

if ~res
   error('Failed to find a feasible initial solution!'); 
end

% compute reachable set for the initial solution
Opts.tStart = 0;
[param,res] = reachSetHorizon(sys,Opts.R0,param,Opts);

if ~res
   error('Failed to find a feasible initial solution!'); 
end


%% Algorithm

% implementation of Alg. 1 in [1]

p = x0;                 % current measurement

reach.R = {Opts.R0};    % time point reachable set
reach.Rcont = [];       % time interval reacahble set    
sim.t = [];             % real system trajectory (time)
sim.x = [];             % real system trajectory (states)
sim.u = [];             % real system trajectory (inputs)

counter = 1;

while length(param.K) >= counter
    
    % simulation of the real system behaviour (for allocated comp. time)
    [x,t,u] = simulateSys(sys,param.K{counter},param.uc(:,counter),...
                        param.xc(:,counter),p,Opts.tComp,Opts);
                    
    sim.x = [sim.x;x(:,Opts.nx+1:end)];
    sim.u = [sim.u;u];
    if isempty(sim.t)
        sim.t = [sim.t;t];
    else
        Opts.tStart = sim.t(end);
        sim.t = [sim.t;sim.t(end)+t];
    end
    
    % reachable set computation (for allocated comp. time)
    tic
    [Rfin,R] = compReachSet(sys,param.K{counter},param.uc(:,counter),...
                         param.xc(:,counter),p,Opts);
             
    % compute new solution         
    [param_,res] = optimalControlReachsetMPC(center(Rfin),param.J,param.L,Opts); 
    
    if res
        Opts.tStart = Opts.tStart + Opts.tComp;
        [param_,res] = reachSetHorizon(sys,Rfin,param_,Opts);
    end
    tComp = toc;
    
    % switch to new solution (see Lines 6-9 of Alg. 1 in [1])
    if res
        resJset = param_.Jset-param.Jset <= -Opts.alpha;
        resTime = tComp < Opts.tComp;
    end
    
    if res && resJset && (resTime || ~Opts.realTime)
       
        disp('Switch to new solution');
        counter = 1;
        param = param_;
        
    else
        disp('Keep old solution');
        counter = counter + 1;
    end
    
    % simulation of the real system behaviour (whole time step)
    if res && resJset && (resTime || ~Opts.realTime)
       
        reach.R{end+1} = Rfin;
        reach.Rcont = add(reach.Rcont,R);
        
    else
        
        [x,t,u] = simulateSys(sys,param.K{counter-1}, ...
                            param.uc(:,counter-1),x(end,Opts.nx+1:end)', ...
                            sim.x(end,:)',Opts.dT-Opts.tComp,Opts);
        
        sim.x = [sim.x;x(:,Opts.nx+1:end)];
        sim.t = [sim.t;sim.t(end)+t];    
        sim.u = [sim.u;u];           
                        
        reach.R{end+1} = param.R{counter-1};
        reach.Rcont = add(reach.Rcont,param.Rcont(counter-1));
    end
    
    % update measurement and time
    p = sim.x(end,:)';
end


%% Assign output values
         
reach.Rcont = projectReachSet(reach.Rcont,1:Opts.nx);

res = results(reach.Rcont,reach.R,[],{sim});

end


% Auxiliary Functions -----------------------------------------------------

function [Opts,sys] = initialization(benchmark,Param,Opts)

    % function handle to dynamic file
    str = ['funHan = @(x,u,w)',benchmark,'(x,u,w);'];
    eval(str);
    
    % get number of states, inputs, and disturbances
    if isfield(Opts,'nx') && isfield(Opts,'nu') && isfield(Opts,'nw')
        nx = Opts.nx; nu = Opts.nu; nw = Opts.nw;
        Opts = rmfield(Opts,'nx');
        Opts = rmfield(Opts,'nu');
        Opts = rmfield(Opts,'nw');
    else
        [count,out] = numberOfInputs(funHan,3);

        nx = out;
        nu = count(2);
        nw = count(3);
    end
    
    % check if the specified paramters are valid
    checkParam(Param,mfilename,nx,nu,nw);
    Opts = checkOpts(Opts,mfilename,Param,nx,nu);

    % copy benchmark parameter to options
    Opts = params2options(Param,Opts);
    Opts.funHandle = funHan;
    Opts.nx = nx;
    Opts.nu = nu;
    Opts.nw = nw;
    
    if isfield(Opts.cora,'alg') && strcmp(Opts.cora.alg,'poly')
        Opts.R0 = polyZonotope(Opts.R0);
    else
        Opts.R0 = zonotope(Opts.R0);
    end  
    
    % options for reachability analysis with the CORA toolbox
    Opts.ReachOpts = Opts.cora;
    Opts = rmfield(Opts,'cora');
    Opts.ReachParams.U = zonotope(Opts.W);

    % store important parameter
    Opts.dT = Opts.tOpt/Opts.N;
    Opts.uMin = infimum(Opts.U);
    Opts.uMax = supremum(Opts.U);
    Opts.uMin_ = infimum(Opts.U_);
    Opts.uMax_ = supremum(Opts.U_);
    Opts.ReachOpts.timeStep = Opts.dT / Opts.reachSteps;
    Opts.ReachOpts.lagrangeRem.simplify = 'optimize';
    
    % store parameter for terminal region
    poly = Opts.termReg;
    temp.set = poly;
    temp.A = get(poly,'A');
    temp.b = get(poly,'b');
    Opts.termReg = temp;

    % create function handles for matrices of the linearized system
    x = sym('x',[Opts.nx,1]);
    u = sym('u',[Opts.nu,1]);
    w = sym('w',[Opts.nw,1]);
    
    Asym = jacobian(Opts.funHandle(x,u,w),x);
    Bsym = jacobian(Opts.funHandle(x,u,w),u);
    
    Asym = subs(Asym,w,zeros(Opts.nw,1));
    Bsym = subs(Bsym,w,zeros(Opts.nw,1));
    
    Opts.A = matlabFunction(Asym,'Vars',{x,u});
    Opts.B = matlabFunction(Bsym,'Vars',{x,u});
    
    % closed loop dynamics   
    name = ['AROCreachMPC',benchmark];
    
    sys = nonlinParamSys(name, ...
            @(x,u,p) closedLoopTracking(x,u,p,Opts.funHandle,Opts.nu), ...
            2*Opts.nx,Opts.nw,(1+Opts.nx)*Opts.nu);
    
    % generate ACADO files
    if isempty(which('BEGIN_ACADO'))
        Opts.useAcado = 0;
    else
        Opts.useAcado = 1;

        w = warning();
        warning('off','all');

        [path,~,~] = fileparts(mfilename('fullpath'));
        rmpath(genpath(fullfile(path,'acado')));

        acadoConvertDynamics(path,Opts,benchmark);
        writeAcadoFilesReachsetMPC(path,benchmark,Opts);

        warning(w);
    end
    
    % pre-compute tensors required for reachability analysis
    initializeReach(sys,Opts);
end

function initializeReach(dyn,Opts)
% exectue reachability analysis once to generate the files for tensors in
% CORA

    % reachability settings
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    
    params.paramInt = zeros((Opts.nx+1)*Opts.nu);
    params.R0 = zonotope([center(Opts.R0);center(Opts.R0)]);
    params.tFinal = Opts.tOpt/Opts.N/Opts.reachSteps;
    
    options.timeStep = params.tFinal;

    % reachability analysis
    try
        evalc('Rcont = reach(dyn,params,options);');
    catch
        Rcont = []; 
    end
end