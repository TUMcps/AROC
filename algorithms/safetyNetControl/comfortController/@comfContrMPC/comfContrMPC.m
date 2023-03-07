classdef comfContrMPC
% COMFCONTRMPC - class representing a MPC controller object 
%
% Syntax:
%       obj = COMFCOnTRMPC(benchmark,Opts,ContrOpts)
%
% Description:
%       This class represents a MPC (Model Predictive Control) controller
%       object which can be used as a comfort controller during the online
%       phase of the Safety Net Controller.
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/...")
%       -Opts:              a structure containing following options
%
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal set should be
%                           reached
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%           -.N:            number of time-steps
%                           [{10} / positive integer]
%           -.xCenter:      reference trajectory states
%           -.uCetner:      reference trajectory inputs
%
%       -ContrOpts:         a structure with options for comfort controller
%   
%           -.Q:            state weighting matrix for the MPC controller
%           -.R:            input weighting matrix for the MPC controller
%           -.horizon:      optimization horizon for the optimal control
%                           problems in reference trajectory time steps
%           -.Ninter:       number of intermediate time steps during one
%                           reference trajectory time step
%
% Output Arguments:
%
%       -obj:   resulting object of class comfContrMPC
%
% See Also:
%       safetyNetControl, objSafetyNetContr
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
    
properties (SetAccess = protected, GetAccess = public)
    xf = [];            % final state
    tFinal = [];        % final time
    U = [];             % set of admissble control inputs
    W = [];             % set of disturbances
    V = [];             % set of measurement errors
    X = [];             % set of state constraints
    N = [];             % number of time steps for safety net controller
    Ninter = [];        % number of intermediate time steps
    Npred = [];         % number of time step during alloc. comp. time
    xCenter = [];       % reference trajectory states
    uCenter = [];       % reference trajectory inputs
    horizon = [];       % length of the optimization horizon
    OptContrOpts = [];  % settings for solving optimal control problems
    ReachOpts = [];     % reachability options for CORA toolbox
    ReachParams = [];   % reachability parameter for CORA toolbox
    sys = [];           % nonlinSys object with system dynamics
    nx = [];            % number of states
    nu = [];            % number of inputs
    nw = [];            % number of disturbances
    path = [];          % file path to the ACADO files
    benchmark = [];     % name of the bechmark
    funHandle = [];     % function handle to the dynamic function
end
   
methods

    function obj = comfContrMPC(benchmark,Opts,ContrOpts)
    % object constructor
        
        % copy options from input arguments
        obj.benchmark = benchmark;
        
        obj.xf = Opts.xf;
        obj.tFinal = Opts.tFinal;
        obj.U = Opts.U;
        obj.W = Opts.W;
        obj.V = Opts.V;
        obj.X = Opts.X;
        obj.N = Opts.N;
        obj.xCenter = Opts.xCenter;
        obj.uCenter = Opts.uCenter;
        
        % store useful parameters
        obj.nx = length(obj.xf);
        obj.nu = dim(obj.U);
        obj.nw = dim(obj.W);
        
        % check if settings for comfort controller are valid
        checkOpts(obj,ContrOpts,Opts);
        
        obj.horizon = ContrOpts.horizon;
        obj.Ninter = ContrOpts.Ninter;
        
        % check if allocated computation time is valid
        obj.Npred = Opts.tComp/(Opts.tFinal/(Opts.N*obj.Ninter))+1;
        
        % set options for reachability analysis with the CORA toolbox
        [params,options] = coraOptions(obj);
        obj.ReachOpts = options;
        obj.ReachParams = params;
        
        % construct the system dynamics of the controlled system
        str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
        eval(str);
        
        funHanMPC = @(x,w) dynamicsClosedLoopLinear(x,w,obj.nx, ...
                                                      obj.nu,funHandle);
                                           
        obj.sys = nonlinearSys(@(x,w) funHanMPC(x,w),obj.nx+obj.nu,obj.nw);                
        obj.funHandle = funHandle;
        
        % set options for solving optimal control problems
        obj.OptContrOpts = optimalControlOptions(obj,ContrOpts);
    end    
    
    function [params,options] = coraOptions(obj)
    % define the settings for the CORA reachability analysis toolbox

        % set of disturbances and final time for reachability analysis
        params.U = zonotope(obj.W);
        params.tFinal = obj.tFinal/(obj.N*obj.Ninter);   

        % set options for CORA toolbox      
        options.taylorTerms = 4;         
        options.zonotopeOrder = 10;
        options.lagrangeRem.simplify = 'optimize';

        options.timeStep = params.tFinal;            

        % additional parameters for reachability analysis
        options.alg = 'lin';
        options.tensorOrder = 2;
    end
    
    function Opts = optimalControlOptions(obj,ContrOpts)
    % define the settings for solving optimal control problems
        
        % set options for optimal control problems
        Opts.Nc = obj.N*obj.Ninter;
        Opts.nx = obj.nx;
        Opts.nu = obj.nu;
        Opts.nw = obj.nw;
        Opts.hc = obj.tFinal/Opts.Nc;
        Opts.xf = obj.xf;
        Opts.uMax = supremum(obj.U);
        Opts.uMin = infimum(obj.U);
        Opts.extHorizon.active = 0;
        Opts.extHorizon.horizon = 1;
        Opts.extHorizon.decay = 'uniform';
        
        % check if ACADO toolbox is installed
        if isempty(which('BEGIN_ACADO'))
            Opts.useAcado = 0;
        else
            Opts.useAcado = 1;
        end
        
        % store state and input weightening matrix
        Opts.Q = ContrOpts.Q;
        Opts.R = ContrOpts.R;
    end
    
    function checkOpts(obj,ContrOpts,Opts)
    % check if the settings for the comfort controller are valid   
        
        % determine redundant fields
        redFields = getRedundantFields(ContrOpts, ...
                                {'Q','R','horizon','Ninter'},'contrOpts');
        
        if ~isempty(redFields)
            text = redFields{1};
            for i = 2:length(redFields)
               text = [text,', ',redFields{i}]; 
            end
            warning(['The following fields of struct "Opts" are redundant: ',text]);
        end
        
        % check state weighting matrix
        if ~isfield(ContrOpts,'Q')
           error('State weighting matrix "Opts.contrOpts.Q" is missing!');
        elseif ~isnumeric(ContrOpts.Q) || ~all(size(ContrOpts.Q) == [obj.nx,obj.nx])
           error('Wrong value for "Opts.contrOpts.Q"! Has to be a square matrix with dimensions matching the system states!');
        else
           if isdiag(ContrOpts.Q)
              if any(ContrOpts.Q < 0)
                  error('Wrong value for "Opts.contrOpts.Q"! Has to be a positive semi-definite matrix!');
              end
           else
              % check if matrix is positive semi-definite
              [~,D] = eig(ContrOpts.Q);
              if ~isreal(D) || any(D < 0)
                  error('Wrong value for "Opts.contrOpts.Q"! Has to be a positive semi-definite matrix!');
              end
           end
        end
        
        % check input weighting matrix
        if ~isfield(ContrOpts,'R')
           error('State weighting matrix "Opts.contrOpts.R" is missing!');
        elseif ~isnumeric(ContrOpts.R) || ~all(size(ContrOpts.R) == [obj.nu,obj.nu])
           error('Wrong value for "Opts.contrOpts.R"! Has to be a square matrix with dimensions matching the system inputs!');
        else
           if isdiag(ContrOpts.R)
              if any(ContrOpts.R < 0)
                  error('Wrong value for "Opts.contrOpts.R"! Has to be a positive semi-definite matrix!');
              end
           else
              % check if matrix is positive semi-definite
              [~,D] = eig(ContrOpts.R);
              if ~isreal(D) || any(D < 0)
                  error('Wrong value for "Opts.contrOpts.R"! Has to be a positive semi-definite matrix!');
              end
           end
        end
        
        % check MPC horizon
        if ~isfield(ContrOpts,'horizon')
            error('MPC optimization horizon "Opts.contrOpts.horizon" is missing!');
        elseif ~isscalar(ContrOpts.horizon) || ContrOpts.horizon < 1 || ...
                    ~mod(ContrOpts.horizon,1) == 0
            error('Wrong value for "Opts.contrOpts.horizon"! Has to be an integer >= 1!');
        end
        
        % check number of intermediate time steps
        if ~isfield(ContrOpts,'Ninter')
            error('Number of intermediate time steps "Opts.contrOpts.Ninter" is missing!');
        elseif ~isscalar(ContrOpts.Ninter) || ...
                      ContrOpts.Ninter < 1 || ~mod(ContrOpts.Ninter,1) == 0
            error('Wrong value for "Opts.contrOpts.Ninter"! Has to be an integer >= 1!');
        end
        
        % check allocated computation time
        temp = Opts.tComp/(obj.tFinal/(obj.N*ContrOpts.Ninter));
        if abs(temp - round(temp)) > 1e-12
            error('Wrong value for "Opts.tComp"! Has to be a multiple of the comfort controller time step!');
        end
    end
end
end