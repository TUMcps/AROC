function [objContr,res] = polynomialControl(benchmark,Param,Opts,varargin)
% POLYNOMIALCONTROL - Implementation of polynomial controller synthesis
%
% Syntax:
%       [objContr,res] = POLYNOMIALCONTROL(benchmark,Param,Opts)
%       [objContr,res] = POLYNOMIALCONTROL(benchmark,Param,Opts,Post)
%
% Description:
%       Offline-phase computations for the polynomial controller synthesis 
%       algorithm.
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
%           -.N:                number of time-steps
%                               [{10} / positive integer]
%           -.Ninter:           number of intermediate timesteps between
%                               two center trajectory timesteps
%                               [{4} / positive integer]
%           -.ctrlOrder:        polynomial order for the controller
%                               [{2} / positive integer]
%           -.reachSteps:       number of reachability steps during one 
%                               time step (optimization)
%                               [{10} / positive integer]
%           -.reachStepsFin:    number of reachability steps during one
%                               time step (final reachable set computation)
%                               [{20} / positive integer]
%           -.Q:                state weighting matrix for the cost 
%                               function of the optimization problem
%           -.splits:           number of recursive splits used to refine
%                               the bounds for the control parameters
%                               [{0} / positive integer]
%           -.refInput:         use the input from the reference trajectory
%                               as input for the center instead of 
%                               optimizing
%                               [{true} / boolean]
%           -.refUpdate:        update the reference trajectory after each
%                               time step
%                               [{false} / boolean]
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
%                                    'fallEqDiff' / 'FallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
%           
%           -.refTraj.Q:    state weighting matrix for the cost function of
%                           the optimal control problem
%           -.refTraj.R:    input weighting matrix for the cost function of
%                           the optimal control problem
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,N*Ninter + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,N*Ninter])
%
%
%           -Post:          function handle to the postprocessing function 
%                           that is used to compute the occupancy set
%
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objPolyContr)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objPolyContr
%
% References:
%       * *[1] Gassman et al. (2021)*, Verified Polynomial Controller 
%              Synthesis for Disturbed Nonlinear Systems
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Syntesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


%% Initialization

Opts = initialization(benchmark,Param,Opts);


%% Center Trajectory

dyn = Opts.funHandle;

if Opts.refUpdate
    xref = zeros(Opts.nx,Opts.N*Opts.Ninter+1);
    xref(:,1) = center(Opts.R0);
    uref = zeros(Opts.nu,Opts.N*Opts.Ninter);
else
    dT = Param.tFinal;
    centOpts = setupCentOpts(Opts.N*Opts.Ninter,1,center(Opts.R0),...
                            Param.xf,dT,Opts.CentOpts);
    [xref,uref] = centerTrajectory(dyn,centOpts);
end


%% Algorithm

% setup template polynomial zonotopes for the control law
% We parameterize alpha and not u (u = cu +Gu*alpha)
[Alpha_scalar,Alpha] = setupControlTemplates(Opts);
P = splitControlTemplates(Alpha_scalar,[Opts.idc;Opts.idp_s],Opts.splits);

% initialization
R0 = polyZonotope(Opts.R0);
Ctrl_x = cell(Opts.N,1);
parallelo = cell(Opts.N,1);
R = [];
p0_ = zeros(Opts.Np*Opts.N,1);      % initial guess for parameter value

% loop over all time steps
for i = 1:Opts.N
    fprintf('Time step: %i\n',i);

    % extended horizon update
    Opts = updateExtHorizon(i,Opts);
    len = Opts.extHorizon.length;
    
    % adapt length of the initial guess
    p0_ = [p0_(Opts.Np+1:len*Opts.Np);zeros(Opts.Np,1)];
    
    % enclose reachable set by a parallelotope
    Z_P = reduce(zonotope(R0),'pca',1);
    parallelo{i} = Z_P;
    GP_inv = inv(generators(Z_P));
    cP = center(Z_P);

    % restructure polynomial zonotope
    R0 = restructure(R0,'reducePca',10,100);
    Opts.idv = R0.id;
    cx = center(R0);    
    
    % update reference trajectory
    if Opts.refUpdate

        % compute new reference trajectory starting from the current state
        dT = Param.tFinal - Param.tFinal/Opts.N * (i-1);
        centOpts = setupCentOpts((Opts.N-i+1)*Opts.Ninter,1,cx,Param.xf,...
                                 dT,Opts.CentOpts);

        [x_c,u_c] = centerTrajectory(dyn,centOpts);

        % add to final reference trajectory (exclude previous point)
        xref(:,(i-1)*Opts.Ninter+2:i*Opts.Ninter+1) = x_c(:,2:Opts.Ninter+1);
        uref(:,(i-1)*Opts.Ninter+1:i*Opts.Ninter) = u_c(:,1:Opts.Ninter);

    else
        x_c = xref(:,(i-1)*Opts.Ninter+(1:len*Opts.Ninter+1));
        u_c = uref(:,(i-1)*Opts.Ninter+(1:len*Opts.Ninter));
    end

    Opts.Xf = x_c(:,1:Opts.Ninter:end);
    
    % factors alpha for the center trajectory
    alpha_c = generators(Opts.U)\(u_c-center(Opts.U));

    % extract the required values from the reference trajectory
    u_c = u_c(:,1:len*Opts.Ninter);
    alpha_c = alpha_c(:,1:len*Opts.Ninter);
    
    % update controller templates
    depfac_diff = 0*GP_inv*(cP-cx);
    
    % w in dependence of v
    pZw_v = GP_inv*(-cP+R0);
    % w in dependence of x
    pZw_x = polyZonotope(-GP_inv*cP,GP_inv,zeros(Opts.nx,0), ...
                                                eye(Opts.nx),Opts.idx);

    Alpha_tmp = resolve(Alpha,depfac_diff,Opts.idc);

    Alpha_x = subs(Alpha_tmp,pZw_x,Opts.idw);
    Alpha_v = subs(Alpha_tmp,pZw_v,Opts.idw);

    U_x = generators(Opts.UU)*Alpha_x + center(Opts.UU);    % u(x)
    U_v = generators(Opts.UU)*Alpha_v + center(Opts.UU);    % u(v)
      
    Opts.P = resolveControlTemplates(P,depfac_diff,Opts);

    % parametric reachability analysis for the overall parameter set
    Rparam = reachPolyParam(R0,U_v,cx,u_c,Opts);

    % optimize to determine the best control law parameters
    pval_ = computeCtrl(p0_,Rparam,alpha_c,Opts);

    pval = pval_(1:Opts.Np);    % only use first Np vals
    p0_ = pval_;                % update initial guess for next iteration

    % compute final closed-loop reachable set
    pZCtrl_x = resolve(U_x,pval,Opts.idp);
    Ctrl_x{i} = pZCtrl_x;
    pZCtrl = subs(pZCtrl_x,R0,Opts.idx);

    if ~isempty(Opts.V)
        tmp = subs(-center(pZCtrl_x)+pZCtrl_x,polyZonotope(Opts.V),Opts.idx);
        pZCtrl = pZCtrl + zonotope(tmp);
    end
 
    [Rtmp,R0] = reachPoly(R0,pZCtrl,Opts);

    R = add(R,Rtmp);
end


%% Assign output values

% check if the state constraints are satisfied
if isfield(Param,'X')
    checkStateConstraints(R,Param.X);
end

% create results object
Rfin = query(R,'finalSet');

if ~iscell(Rfin)
    Rfin = {Rfin};
end

refTraj.xc = xref;
refTraj.uc = uref;

res = results(R,[{polyZonotope(Param.R0)};Rfin],refTraj);

% compute the occupancy set
occSet = [];

if nargin > 3
   post = varargin{1};
   occSet = compOccupancySet(R,post,Opts);
end

% create controller object
contrLaw.N = Opts.N;
contrLaw.Ninter = Opts.Ninter;
contrLaw.Ctrl_x = Ctrl_x;
contrLaw.parallelo = parallelo;

objContr = objPolyContr(Opts.funHandle,Rfin{end},contrLaw,Param,occSet);
end


% Auxiliary Functions -----------------------------------------------------

function Opts = initialization(benchmark,Param,Opts)
% initialize the settings for the control algorithm
    
    % initialize general options
    Opts = initOpts(mfilename,benchmark,Opts,Param);

    % time step size for reachability analysis
    Opts.dt = Opts.tFinal/(Opts.N*Opts.Ninter);
    Opts.ReachOpts.timeStep = Opts.dt / Opts.reachStepsFin;
    
    % reachability settings for optimization
    Opts.cost.ReachOpts = Opts.ReachOpts;
    Opts.cost.ReachOpts.timeStep = Opts.dt / Opts.reachSteps;
    Opts.cost.ReachOpts.approxDepOnly = true;
    Opts.cost.ReachOpts.reductionTechnique = ...
                ['approxdep_',Opts.cost.ReachOpts.reductionTechnique];

    Opts.cost.ReachParams = Opts.ReachParams;
    Opts.cost.ReachParams.U = 0*Opts.cost.ReachParams.U;

    % exponential matrix for the polynomial controller
    Opts.eMCtrl = generateExpMat(Opts.nx,Opts.ctrlOrder);
    
    % number of parameters per input dimension
    Opts.np_s = size(Opts.eMCtrl,2);

    % number of overall paraemters
    Opts.np = Opts.nu*Opts.np_s;
    Opts.Np = Opts.np*Opts.Ninter;

    % identifiers for states
    Opts.idx = -(1:Opts.nx)';
    max_id_abs = max(abs(Opts.idx)); 

    % identifiers for parameters for all inputs of the extended horizon
    Opts.Idp = reshape(-(max_id_abs+(1:Opts.N*Opts.Np)),[Opts.Np,Opts.N]);
    max_id_abs = max(abs(Opts.Idp(:)));

    % identifiers for parameters beloning to scalar inputs
    Opts.idp_s = Opts.Idp(1:Opts.np_s,1);

    % identifiers for parameters beloning to single inputs
    Opts.idp_j = Opts.Idp(1:Opts.np,1);

    % identifiers for parameters for all inputs j in {1,...,Ninter}
    Opts.idp = Opts.Idp(:,1);

    Opts.idw = -(max_id_abs+(1:Opts.nx))';
    max_id_abs = max(abs(Opts.idw));
    Opts.idc = -(max_id_abs+(1:Opts.nx))';
    
    % system dynamics of the controlled system
    dyn_handle = eval(['@',benchmark]);
    extDyn = @(x,w)[dyn_handle(x(1:Opts.nx),x(Opts.nx+1:Opts.nx+Opts.nu),w);
                    zeros(Opts.nu,1)];
    
    name = ['AROCPGSC',benchmark,Opts.ReachOpts.alg, ...
            num2str(Opts.ReachOpts.tensorOrder)];
    
    Opts.sys = nonlinearSys(name,extDyn,Opts.nx+Opts.nu,Opts.nw);
    
    % control input zonotope for multiple time steps
    tmp = repmat({generators(Opts.U)},Opts.Ninter,1);
    G = blkdiag(tmp{:});
    c = repmat(center(Opts.U),Opts.Ninter,1);

    Opts.UU = zonotope([c,G]);
end

function centOpts = setupCentOpts(N,nr,x0,xf,dT,Def)
% define options for reference trajectory computation

    centOpts = Def;
    centOpts.Nc = N;
    centOpts.hc = dT/N;
    centOpts.Nr = nr;
    centOpts.x0 = x0;
    centOpts.xf = xf;
    centOpts.tf = dT;
end