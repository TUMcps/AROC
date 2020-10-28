function [c,ceq] = conFun(x,Opts)
% CONFUN - constraint function for nonlinear programming
%
% Syntax:
%       [c,ceq] = CONFUN(x,Opts)
%
% Description:
%       This function computes the values of the constraints for a specific 
%       value for the control law parameters. For this, the reachable set 
%       is computed fist. Afterward, it is calculated how much the
%       reachable set violates the constraints.
%
% Input Arguments:
%
%       -x:     current value of the control law parameters
%       -Opts:  structure containing the following options
%
%           -.linModel:     structure containing the system matrices of the
%                           uncontrolled system
%           -.ReachOpts:    structure containing the settings for
%                           reachability analysis
%           -.uc:           center trajectory control inputs
%
% Output Arguments:
%       -c:     amount of violation for the inequality constraints 
%               (c <= 0: no violation)
%       -ceq:   amount of violation for the equality constraints
%               (c == 0: no violation)
%
% See Also:
%       optimizationBasedControl, costFun
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

    c = [];

    % get options for reachability analysis
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    
    params.R0 = cartProd(Opts.R0,zonotope(Opts.x0));
    
    % loop over all time steps
    for i = 1:Opts.N
        
        % get reference input
        u_ref = Opts.uc(:,i);
            
        % compute closed loop system
        if Opts.isLin
            
            % get feedback matrix
            K = compFeedbackMatrix(x(:,i),Opts.linModel.A, ...
                                   Opts.linModel.B);
            
            % construct linear system
            sys = closedLoopSystemLin(Opts.linModel.A,Opts.linModel.B, ...
                                      Opts.linModel.D,Opts.linModel.c, ...
                                      K,u_ref);
                                  
            % compute the reachable set
            R = reach(sys,params,options);
            
        else
            
            % get feedback matrix
            K = compFeedbackMatrix(x(:,i),Opts.linModel.A{i}, ...
                                   Opts.linModel.B{i});
            
            p = [u_ref;reshape(K,[Opts.nx*Opts.nu,1])];
            params.paramInt = p;
            
            % compute the reachable set
            try
                R = reachNonlinear(Opts.sys,params,options);
            catch
                if isempty(Opts.stateCon)
                    M = size(Opts.inputCon.A,1);
                else
                    M = size(Opts.inputCon.A,1)+size(Opts.stateCon.A,1);
                end
                c = inf * ones(M*Opts.N*Opts.reachSteps,1);
                ceq = [];
                return;
            end
        end
        
        % loop over all reachability time steps
        cTemp = checkConstraints(R.timeInterval.set,K,u_ref,Opts);
        c = [c;cTemp];
        
        % update initial set
        params.R0 = R.timePoint.set{end};
    end
    
    % assign output values
    ceq = [];   
end