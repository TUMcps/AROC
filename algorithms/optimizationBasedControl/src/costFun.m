function cost = costFun(x,Opts)
% COSTFUN - cost function for nonlinear programming
%
% Syntax:
%       cost = COSTFUN(x,Opts)
%
% Description:
%       This function computes the costs for a specific value for the 
%       control law parameters. For this, the reachable set is computed and
%       the distance of the final reachable set to the target state is
%       computed as the cost.
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
%
%       -cost:  value of the cost function
%
% See Also:
%       optimizationBasedControl, conFun
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
                cost = inf;
                return;
            end
        end
        
        % update initial set
        Rfin = R.timePoint.set{end};
        params.R0 = Rfin;
    end
    
    % calculate costs (slightly modified from [1])
    int = interval(project(Rfin,1:Opts.nx));
    
    r = rad(int);
    c = center(int);

    cost = sum(r.^2./rad(interval(Opts.R0)).^2) + sum(abs(Opts.xf - c));
end