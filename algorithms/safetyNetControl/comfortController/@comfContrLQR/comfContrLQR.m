classdef comfContrLQR
% COMFCONTRLQR - class representing an LQR controller object 
%
% Syntax:
%       obj = CONFCONTRLQR(benchmark,Opts,ContrOpts)
%
% Description:
%       This class represents a LQR (Linear Quadratic Regulator) controller
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
%           -.Ninter:       number of intermediate time steps
%                           [{4} / positive integer]
%           -.uCenter:      reference trajectory inputs
%           -.xCenter:      reference trajectory states

%
%       -ContrOpts:         a structure with options for comfort controller
%   
%           -.Q:            State weighting matrix for the LQR controller
%           -.R:            Input weighting matrix for the LQR controller
%
% Output Arguments:
%
%       -obj:   resulting object of class comfContrLQR
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
    N = [];             % number of time steps
    Ninter = [];        % number of intermediate time steps
    Npred = [];         % number of time step during alloc. comp. time
    uCenter = [];       % reference trajectory inputs
    xCenter = [];       % reference trajectory states
    K = [];             % feedback matrices
    ReachOpts = [];     % reachability options for CORA toolbox
    ReachParams = [];   % reachability parameter for CORA toolbox
    sys = [];           % nonlinSys object with system dynamics
    nx = [];            % number of states
    nu = [];            % number of inputs
    nw = [];            % number of disturbances
end
   
methods

    function obj = comfContrLQR(benchmark,Opts,ContrOpts)
    % object constructor
        
        % copy options from input arguments
        obj.xf = Opts.xf;
        obj.tFinal = Opts.tFinal;
        obj.U = mptPolytope(Opts.U);
        obj.W = Opts.W;
        obj.V = Opts.V;
        obj.X = Opts.X;
        obj.N = Opts.N;
        obj.Ninter = Opts.Ninter;
        obj.uCenter = Opts.uCenter;
        obj.xCenter = Opts.xCenter;
        
        % store useful parameters
        obj.nx = length(obj.xf);
        obj.nu = dim(obj.U);
        obj.nw = dim(obj.W);
        
        % check if settings for comfort controller are valid
        checkOpts(obj,ContrOpts,Opts);
        
        % number of time steps during allocated computation time
        obj.Npred = Opts.tComp/(Opts.tFinal/(Opts.N*Opts.Ninter))+1;
        
        % set options for reachability analysis with the CORA toolbox
        [options,params] = coraOptions(obj);
        obj.ReachOpts = options;
        obj.ReachParams = params;
        
        % construct the system dynamics of the controlled system
        str = ['funHandle = @(x,u,w)',benchmark,'(x,u,w);'];
        eval(str);
        
        if isempty(obj.V)
           funHanLQR = @(x,w,p) closedLoopTracking(x,w,p,funHandle,obj.nu);
                                           
           obj.sys = nonlinParamSys(@(x,w,p) funHanLQR(x,w,p),2*obj.nx, ...
                                                obj.nw,(obj.nx+1)*obj.nu);
        else
           funHanLQR = @(x,w,p) closedLoopTrackingMeasErr(x,w,p, ...
                                                         funHandle,obj.nu);
                                           
           obj.sys = nonlinParamSys(@(x,w,p) funHanLQR(x,w,p),2*obj.nx, ...
                                          obj.nw+obj.nx,(obj.nx+1)*obj.nu); 
        end

        % compute the feedback matrices for all time steps
        obj.K = initFeedbackMatrices(obj,funHandle,ContrOpts.Q,ContrOpts.R);
    end    
    
    function [options,params] = coraOptions(obj)
    % define the settings for the CORA reachability analysis toolbox

        % set of disturbances and final time for reachability analysis
        params.U = zonotope(obj.W);
        if ~isempty(obj.V)
           params.U = cartProd(params.U,obj.V); 
        end
        params.tFinal = obj.tFinal/(obj.N*obj.Ninter); 

        % set options for CORA toolbox        
        options.taylorTerms = 4;         
        options.zonotopeOrder = 10;       
        options.intermediateTerms = 4;
        options.lagrangeRem.simplify = 'optimize';

        options.timeStep = params.tFinal;            

        % additional parameters for reachability analysis
        options.alg = 'lin';
        options.tensorOrder = 2;
        
        % parameter interval
        params.paramInt = zeros((obj.nx+1)*obj.nu,1);
    end
    
    function K = initFeedbackMatrices(obj,funHandle,Q,R)
    % initialize the feedback matrices for all time steps    
        
        K = cell(obj.N,1);
    
        % generate function handles for linearized dynamics
        x = sym('x',[obj.nx,1]); 
        u = sym('u',[obj.nu,1]);
        w = sym(zeros(obj.nw,1));

        f = funHandle(x,u,w);

        A = jacobian(f,x);
        B = jacobian(f,u);

        AfunHan = matlabFunction(A,'Vars',{x,u});
        BfunHan = matlabFunction(B,'Vars',{x,u});
        
        % loop over all time steps
        for i = 1:obj.N
           
            K{i} = cell(obj.Ninter,1);
            xc = obj.xCenter{i};
            uc = obj.uCenter{i};
            
            % loop over all intermediate time steps
            for j = 1:obj.Ninter
                
                % compute linearization point
                x_ = 0.5*(xc(:,j) + xc(:,j+1));
                u_ = uc(:,j);
                
                % linearize the system
                A = AfunHan(x_,u_);
                B = BfunHan(x_,u_);
                
                % compute feedback matrix using the LQR approach
                [Ktemp,~,~] = lqr(A,B,Q,R);
                Ktemp = -Ktemp';
                
                K{i}{j} = Ktemp;
            end
        end  
    end   
    
    function checkOpts(obj,ContrOpts,Opts)
    % check if the settings for the comfort controller are valid   
        
        % determine redundant fields
        redFields = getRedundantFields(ContrOpts,{'Q','R'},'contrOpts');
        
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
        
        % check allocated computation time
        temp = Opts.tComp/(obj.tFinal/(obj.N*obj.Ninter));
        if abs(temp - round(temp)) > 1e-12
            error('Wrong value for "Opts.tComp"! Has to be a multiple of the comfort controller time step!');
        end
    end
end
end