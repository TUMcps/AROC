function [objContr,res] = combinedControl(benchmark,Param,Opts)
% COMBINEDCONTROL - Implementation of the combined controller
%
% Syntax:
%       [objContr,res] = COMBINEDCONTROL(benchmark,Param,Opts)
%
% Description:
%       Offline-phase computations for the controller that combines initial 
%       state dependent feed-forward part with a feedback controller.
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
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.N:                number of time steps
%                               [{10} / positive integer]
%           -.feedForward:      type of feed-forward controller used
%                               [{'genSpace'} / 'poly'] 
%           -.reachSteps:       number of reachability steps in one time 
%                               step 
%                               [{10} / positive integer]
%           -.reachStepsFin:    number of reachability steps during one
%                               time step (final reachable set computation)
%                               [{50} / positive integer]
%           -.scale:            scaling factor for the tightend input
%                               constraints [{0.9} / scalar between 0 and 1]
%           -.Q:                state weighting matrix for the cost 
%                               function of the optimal control problem
%                               [{eye(nx)} / pos.-definite square matrix]
%           -.R:                input weighting matrix for the cost 
%                               function of the optimal control problem
%                               [{zeros(nu)} / pos.-definite square matrix]
%           -.Qff:              state weighting matrix for feed-forward 
%                               control
%                               [{eye(nx)} / pos.-definite square matrix]
%           -.Rff:              input weighting matrix for feed-forward 
%                               control
%                               [{zeros(nu)} / pos.-definite square matrix]
%           -.finStateCon:      use constraint that the final reachable set
%                               is inside the shifted initial set
%                               [{false} / boolean]  
%           -.maxIter:          maximum number of iterations for
%                               optimization with fmincon 
%                               [{5} / positive integer]
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
% Output Arguments:
%
%       -objContr:  resulting controller storing the data computed during
%                   the offline phase (class: objCombinedContr)
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       objCombinedContr
%
% References:
%       * [1] Schuermann et al. (2020)*, Optimizing Sets of Solutions for 
%              Controlling Constrained Nonlinear Systems
%       * [2] Gassmann et al. (2021)*, Verified Polynomial Controller 
%              Synthesis for Disturbed, Nonlinear Systems
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
% Authors:      Victor Gassmann, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % Initialization
    [Opts,sys] = initialization(benchmark,Param,Opts);
    
    if strcmp(Opts.feedForward,'genSpace')
        % Feedforward Control (see Sec. IV.A in [1])
        [objContr_ff,res_ff] = generatorSpaceControl(benchmark, ...
                                        Opts.ffCtrlParam,Opts.ffCtrlOpts);
    elseif strcmp(Opts.feedForward,'poly')
        % Feedforward Control (see [2])
        [objContr_ff,res_ff] = polynomialControl(benchmark, ...
                                        Opts.ffCtrlParam,Opts.ffCtrlOpts);
    end
    
    % Feedback Control (see Sec. IV.B in [1])
    Opts.refTraj = res_ff.refTraj;
    Opts.RS_ff = res_ff.reachSet;

    [K,RS] = computeFeedbackControl(sys,objContr_ff,Opts);

    % Assign output values
    Rfin = query(RS,'finalSet');
    if strcmp(Opts.ReachOpts.alg,'poly')
       Rfin = [{polyZonotope(Opts.R0)};Rfin]; 
    else
       Rfin = [{zonotope(Opts.R0)};Rfin]; 
    end

    ctrlLaw.K = K;
    ctrlLaw.FeedforwardCtrl = objContr_ff;
    ctrlLaw.linSys.A = Opts.linDyn.A;
    ctrlLaw.linSys.B = Opts.linDyn.B;
    ctrlLaw.refTraj = Opts.refTraj;

    objContr = objCombinedContr(Opts.funHandle,Rfin{end},ctrlLaw,Param);               
    res = results(RS,Rfin,res_ff.refTraj,[]);

end


% Auxiliary Functions -----------------------------------------------------

function [Opts,sys] = initialization(benchmark,Param,Opts)
% initialize algorithm settings

    % initialize general settings
    Opts = initOpts(mfilename,benchmark,Opts,Param);
    Opts.benchmark = benchmark;
    dyn_handle = eval(['@',benchmark]);
    Opts.dynamics = dyn_handle;
    
    % compute linearized system dynamics
    Opts = symLinearize(Opts);
    
    % compute options for feedforward controller
    Opts.ffCtrlOpts.N = 1;
    Opts.ffCtrlOpts.Ninter = Opts.N;
    Opts.ffCtrlOpts.refTraj = Opts.refTraj;
    Opts.ffCtrlOpts.Q = Opts.Qff;
    Opts.ffCtrlOpts.R = Opts.Rff;
        
    % compute parameters for feedforward controller
    Opts.ffCtrlParam = Param;
    Opts.ffCtrlParam.U = center(Param.U) + ...
                            Opts.scale*(Param.U + (-center(Param.U)));
    Opts.ffCtrlParam.W = 0*Param.W;
    
    % store reachabilty options for linear and nonlinear systems
    Opts.dt = Opts.tFinal/Opts.N;
    Opts.R0 = zonotope(Opts.R0);
    Opts.ReachOpts.timeStep = Opts.dt/Opts.reachSteps;

    Opts.ReachOptsLin = Opts.ReachOpts;
    fields = {'errorOrder','intermediateOrder','alg','tensorOrder'};

    for i = 1:length(fields)
        if isfield(Opts.ReachOptsLin,fields{i})
            Opts.ReachOptsLin = rmfield(Opts.ReachOptsLin,fields{i});
        end
    end
    
    Opts.ReachOpts.intermediateTerms = 4;

    % special settins for polynomial feedforward control
    if strcmp(Opts.feedForward,'poly')
        Opts.ffCtrlOpts = rmfield(Opts.ffCtrlOpts,'R');
        Opts.ReachOpts.tensorOrder = 3;
        Opts.ReachOpts.alg = 'poly';
    end
    
    % store matrices for inequality constraints for input and state con
    if isfield(Param,'X')
       Opts.stateCon.A = get(Param.X,'A');
       Opts.stateCon.b = get(Param.X,'b');
    else
       Opts.stateCon = []; 
    end
    
    Opts.inputCon.A = [eye(Opts.nu);-eye(Opts.nu)];
    Opts.inputCon.b = [supremum(Param.U);-infimum(Param.U)];
    
    % constraint that final reachable set has to be in shifted init. set
    if Opts.finStateCon
        poly = mptPolytope(-center(Opts.R0)+Opts.xf + Opts.R0);
        Opts.finCon.A = poly.P.A;
        Opts.finCon.b = poly.P.b;
    else
        Opts.finCon = [];
    end
    
    % store number of inequality constraints during nonlinear optimization
    temp = length(Opts.inputCon.b);
    if ~isempty(Opts.stateCon)
        temp = temp + length(Opts.stateCon.b);
    end
    
    Opts.nIneq = Opts.N*Opts.reachSteps*temp;
    
    if ~isempty(Opts.finCon)
        Opts.nIneq = Opts.nIneq + length(Opts.finCon.b); 
    end
    
    % create nonlinearSys object for the closed-loop dynamics
    name = ['AROCcombined',benchmark,Opts.ReachOpts.alg, ...
            num2str(Opts.ReachOpts.tensorOrder)];
    
    nu = Opts.nu; nx = Opts.nx; nw = Opts.nw;

    if strcmp(Opts.feedForward,'genSpace')

        if isempty(Opts.V)
            np = 2*nu*nx+nx+2*nu;
            f_ext = @(x,w,p) extendedDynamics(x,w,p,nx,nu,...
                               Opts.funHandle,Opts.linDyn.A,Opts.linDyn.B);
            sys = nonlinParamSys(name,...
                   @(x,w,p)f_ext(x,w,p),4*nx,nw,np,'constParam');       
        else
            np = 2*nu*nx+nx+2*nu;
            f_ext = @(x,w,p) extendedDynamicsMeasErr(x,w,p,nx,nu, ...
                       Opts.nw,Opts.funHandle,Opts.linDyn.A,Opts.linDyn.B);
            sys = nonlinParamSys([Opts.benchmark,'_rs_ext'],...
               @(x,w,p)f_ext(x,w,p),5*nx,nw+nx,np,'constParam');
        end

    else
        np = nu*nx+nx+nu;
        f_ext = @(x,w,p) extendedDynamicsPoly(x,w,p,nx,nu,nw,...
                           Opts.funHandle,Opts.linDyn.A,Opts.linDyn.B);
        sys = nonlinParamSys(name,...
               @(x,w,p)f_ext(x,w,p),3*nx+nu,nw+nx,np,'constParam');
    end  
end