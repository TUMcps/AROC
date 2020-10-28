function [res,reachSet,R,K] = computeFinalReachSet(xOpt,Opts)
% COMPUTEFINALREACHSET - compute the reachable set using the optimized
%                        control law parameters
%
% Syntax:
%       [res,reachSet,R,K] = COMPUTEFINALREACHSET(xOpt,Opts)
%
% Description:
%       This function computes the reachable set using the optimized
%       control law parameters from the optimization based control
%       algorithm.
%
% Input Arguments:
%
%       -xOpt:  optimized weigthening matrices
%       -Opts:  structure containing the following options
%
%           -.R0:           intial set (class: zonotope)
%           -.linModel:     structure containing the system matrices of the
%                           uncontrolled system
%           -.sys:          parametric system object storing the
%                           closed-loop dynamics (class: nonlinParamSys)
%           -.ReachOpts:    structure containing the settings for
%                           reachability analysis
%           -.uc:           center trajectory control inputs
%
% Output Arguments:
%
%       -res:       flag specifying if the constraints are satisfied 
%                   (1 if satisfied, 0 if not)
%       -reachSet:  object storing the reachable set (class: reachSet)
%       -R:         cell-array storing the time point reachable set
%       -K:         cell-array storing the feedback matrices for all time 
%                   steps
%
% See Also:
%       optimizationBasedControl
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

    % initialization
    K = cell(Opts.N,1);
    reachSet = [];
    R = cell(Opts.N,1);
    R{1} = Opts.R0;
    res = 1;
    
    % get options for reachability analysis
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    params.R0 = cartProd(Opts.R0,zonotope(Opts.x0));
    tStart = 0;
    dT = params.tFinal;
    
    % decrease time step size to get a more accurate result
    options.timeStep = params.tFinal/Opts.reachStepsFin;
    
    % loop over all time steps
    for i = 1:Opts.N

        % get reference input
        u_ref = Opts.uc(:,i);
            
        % compute closed loop system
        if Opts.isLin
            
            % get feedback matrix
            K{i} = compFeedbackMatrix(xOpt(:,i),Opts.linModel.A, ...
                                      Opts.linModel.B);
            
            % construct linear system
            sys = closedLoopSystemLin(Opts.linModel.A,Opts.linModel.B, ...
                                      Opts.linModel.D,Opts.linModel.c, ...
                                      K{i},u_ref);
        else
            
            % get feedback matrix
            K{i} = compFeedbackMatrix(xOpt(:,i),Opts.linModel.A{i}, ...
                                      Opts.linModel.B{i});
            
            p = [u_ref;reshape(K{i},[Opts.nx*Opts.nu,1])];
            params.paramInt = p;
            
            % get system object
            sys = Opts.sys;
        end
        
        % update time
        if tStart ~= 0
            params.tStart = tStart;
            params.tFinal = params.tStart + dT;
        else
            params.tFinal = dT;
        end
        tStart = params.tFinal;

        % compute the reachable set
        Rtemp = reach(sys,params,options);
        
        % check if the constraints are satisfied
        c = checkConstraints(Rtemp.timeInterval.set,K{i},u_ref,Opts);
        
        if any(c > 0)
           res = 0;
           return;
        end

        % update initial set
        params.R0 = Rtemp.timePoint.set{end};
        
        % store reachable set
        reachSet = add(reachSet,Rtemp);
        R{i+1} = project(params.R0,1:Opts.nx);
    end
    
    % remove extendet states from the reachable set
    reachSet = projectReachSet(reachSet,1:Opts.nx);
end