function [t,x,u] = simulate(obj,x0,w,v,Param,iter)
% SIMULATE - simulates a system controlled with a LQR comfort controller
%
% Syntax:
%       [t,x,u] = simulate(obj,x0,w,Param,iter)
%
% Description:
%       Simulates the closed-loop system for one time step of the LQR
%       (Linear Quadratic Regulator) controller which is used as a comfort 
%       controller during online application of the Safety Net Controller.
%
% Input Arguments:
%
%       -obj:   object of class comfContrLQR storing the control law
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
%       safetyNetControl, reachSet, comfContrLQR
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

    % get values for the current iteration
    xc = obj.xCenter{iter};
    uc = obj.uCenter{iter};
    K = obj.K{iter};

    % initialization
    x_ = [x0;xc(:,1)];
    Nw = size(w,2);
    if ~isempty(obj.V)
       w = [w;v]; 
    end
    timeStep = obj.tFinal/(obj.N*obj.Ninter*Nw);
    x = []; u = []; t = [];
    
    % loop over all intermediate time steps
    for i = 1:obj.Ninter
       
        % pass current feedback matrix and reference input as parameters
        temp = reshape(K{i},[obj.nu*obj.nx,1]);
        p = [temp;uc(:,i)];
        
        % simulate the system
        for k = 1:Nw
            [tTemp,xTemp] = ode45(@(t,x)obj.sys.mFile(x,w(:,k),p),[0 timeStep],x_);
            uTemp = uc(:,i) + K{i}'*(xTemp(:,1:obj.nx) - xTemp(:,obj.nx+1:end))';

            x_ = xTemp(end,:)';
            x = [x;xTemp(:,1:obj.nx)];  u = [u;uTemp'];
            if isempty(t)
                t = [t;tTemp];
            else
                t = [t;t(end)+tTemp];
            end
        end
    end
end