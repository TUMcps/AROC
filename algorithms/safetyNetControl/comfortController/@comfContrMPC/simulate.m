function [t,x,u] = simulate(obj,x0,w,v,Param,iter)
% SIMULATE - simulates a system controlled with a MPC comfort controller
%
% Syntax:
%       [t,x,u] = simulate(obj,x0,w,v,Param,iter)
%
% Description:
%       Simulates the closed-loop system for one time step of the MPC
%       (Model Predictive Control) controller which is used as a comfort 
%       controller during online application of the Safety Net Controller.
%
% Input Arguments:
%
%       -obj:   object of class comfContrMPC storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the reachable set computation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,Nw])
%       -v:     matrix storing the measurement errors over time
%               (dimension: [n,Nw])
%       -Param: control parameters computed during reachability analyis 
%       -iter:  current iteration (=time step) of the Safety Net Controller
%
% Output Arguments:
%
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input 
%               (dimension: [|t|,nu])
%
% See Also:
%       safetyNetControl, reachSet, comfContrMPC
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
    Nw = size(w,2);
    timeStep = obj.tFinal/(obj.N*obj.Ninter*Nw);
    x_ = x0;
    x = []; u = []; t = [];
    
    % loop over all intermediate time steps
    for i = 1:obj.Ninter
        
        % loop over all disturbance values
        for k = 1:Nw
            
            % update initial point
            x0 = [x_;Param(:,i)];
            
            % simulate the system
            [tTemp,xTemp] = ode45(@(t,x)obj.sys.mFile(x,w(:,k)),[0 timeStep],x0);

            % store the results
            x_ = xTemp(end,1:obj.nx)';
            x = [x;xTemp(:,1:obj.nx)];  
            u = [u;ones(size(xTemp,1),1)*Param(:,i)'];
            if isempty(t)
                t = [t;tTemp];
            else
                t = [t;t(end)+tTemp];
            end
        end
    end
end