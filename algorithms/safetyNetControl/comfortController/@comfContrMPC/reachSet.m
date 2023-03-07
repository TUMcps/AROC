function [res,R,Param] = reachSet(obj,x0,iter)
% REACHSET - computes the reachable set for an MPC comfort controller
%
% Syntax:
%       [res,R,Param] = REACHSET(obj,x0,iter)
%
% Description:
%       Computes the reachable set for one time step of the MPC
%       (Model Predictive Control) controller which is used as a comfort 
%       controller during online application of the Safety Net Controller.
%
% Input Arguments:
%
%       -obj:   object of class comfContrMPC storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the reachable set computation
%       -iter:  current iteration (=time step) of the Safety Net Controller
%
% Output Arguments:
%
%       -res:   flag specifying if the constraints are satisfied or not
%       -R:     final reachable set (class: zonotope)
%       -Param: control parameters computed during the online phase which
%               are required for the simulation of the controller later on
%
% See Also:
%       safetyNetControl, simulate, comfContrMPC
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

    % initialization
    res = 1;
    R0 = zonotope(x0);
    options = obj.ReachOpts;
    params = obj.ReachParams;
    Opts = obj.OptContrOpts;

    % compute the remaining optization horizon and the goal state
    Nc = obj.N - iter + 1;
    
    if Nc <= obj.horizon
        Opts.Nc = Nc*obj.Ninter;
    else
        Opts.Nc = obj.horizon*obj.Ninter;
        xc = obj.xCenter{iter+obj.horizon};
        Opts.xf = xc(:,1);
    end
    
    % solve optimal control problem 
    dyn = obj.funHandle;
    Opts.x0 = center(x0);
    
    [~,uCenter] = centerTrajectory(dyn,Opts);
    
    Param = uCenter(:,1:obj.Ninter);
    
    % loop over all intermediate time steps
    for i = 1:obj.Ninter
       
        % construct initial set
        params.R0 = cartProd(R0,zonotope(uCenter(:,i)));
        
        % compute the reachable set
        Rtemp = reachNonlinear(obj.sys,params,options);
        
        % check state constraints
        if isa(obj.X,'mptPolytope')
            for j = 1:length(Rtemp.timeInterval.set)
               if ~contains(obj.X,project(Rtemp.timeInterval.set{j},1:obj.nx))
                   res = 0; return;
               end
            end
        end
        
        % update the initial set
        Rfin = Rtemp.timePoint.set{end};
        R0 = project(Rfin,1:obj.nx);
    end
    
    % assign output values
    R = project(Rfin,1:obj.nx);
end