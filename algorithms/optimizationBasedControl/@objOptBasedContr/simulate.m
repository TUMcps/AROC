function [res,t,x,u] = simulate(obj,x0,w,varargin)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the Convex Interpolation Controller
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,x0,w)
%       [res,t,x,u] = SIMULATE(obj,x0,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the Convex Interpolation Controller for a given initial point
%       and given specific disturbance values over time.
%
% Input Arguments:
%
%       -obj:   object of class objConvInterContr storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,time steps])
%       -v:     matrix storing the values for the measurement error over 
%               time (dimension: [nx,time steps])
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input
%               (dimension: [|t],nu])
%
% See Also:
%       optimizationBasedControl, simulateRandom
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

    % check if the number of specified disturbance vectors is correct
    Nw = 1;
    
    if size(w,2) > 1
        Nw = size(w,2)/(obj.N);
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of ''N''!'); 
        end
    end
    
    % check if the number of specified measurement errors is correct
    if nargin > 3
        v = varargin{1};
        if ~all(size(v) == [obj.nx,size(w,2)])
           error('Number of measurement errors has to be identical to the number of disturbance vectors!');  
        end
        w = [w; v];
    end

    % compute time step 
    timeStep = (obj.tFinal/obj.N);
    
    % simulate each part with constant input
    x_ = [x0;center(obj.R0)];
    x = []; u = []; t = [];
    counter = 1;

    % loop over all center trajectory time steps
    for i = 1:obj.N
        
        % get control law parameter
        K = obj.K{i};
        u_ref = obj.u_ref(:,i);
        
        % simulate the system
        for k = 1:Nw
            
            % construct closed loop dynamics
            if obj.isLin
                dyn = obj.dynamics;
                if nargin > 3
                    sys = closedLoopSystemLin(dyn.A,dyn.B,dyn.D, ...
                                                        dyn.c,K,u_ref,1);
                else
                    sys = closedLoopSystemLin(dyn.A,dyn.B,dyn.D, ...
                                                        dyn.c,K,u_ref,[]);
                end
                closedLoopEqn = @(t,x) sys.A*x + sys.B*w(:,counter) + sys.c;
            else
                p = [u_ref;reshape(K,[obj.nx*obj.nu,1])];
                
                if isempty(obj.V)
                    closedLoopEqn = @(t,x) closedLoopSystemNonlin(x, ...
                        w(:,counter),p,obj.dynamics,obj.nx,obj.nu,obj.nw);
                else
                    closedLoopEqn = @(t,x) closedLoopSystemNonlinMeasErr(x, ...
                        w(:,counter),p,obj.dynamics,obj.nx,obj.nu,obj.nw);
                end
            end
            
            % simulate the system
            [tTemp,xTemp] = ode45(closedLoopEqn,[0 timeStep/Nw],x_);

            % compute the control input (see Eq. (3) in [1])
            uTemp = u_ref + K*(xTemp(:,1:obj.nx) - ...
                               xTemp(:,obj.nx+1:2*obj.nx))';
            
            % store the results
            x_ = xTemp(end,:)';
            x = [x;xTemp(:,1:obj.nx)];
            u = [u;uTemp'];
            if ~isempty(t)
                t = [t;t(end)+tTemp];
            else
                t = [t;tTemp];
            end
            counter = counter + 1;
        end
        
    end
    
    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end
