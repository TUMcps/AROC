function [res,t,x,u] = simulate(obj,x0,w,varargin)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the controller based on combined control
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,x0,w)
%       [res,t,x,u] = SIMULATE(obj,x0,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the controller based on combined control for a given initial  
%       point and given specific disturbance values over time.
%
% Input Arguments:
%
%       -obj:   object of class objCombinedContr storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,multiple of N])
%       -v:     matrix storing the values for the measurement errors over
%               time (dimension: [nx,multiple of N]) 
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the control inputs for the simulated
%               trajectory (dimension: [|t|,nu])
%
% See Also:
%       combinedControl, simulateRandom
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
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % check if the number of specified disturbance vectors is correct
    if size(w,2) > 1
        Nw = size(w,2)/obj.N;
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of "N"!'); 
        end
    end
    
    % check if the number of specified measurement errors is correct
   	if length(varargin) >= 1
       v = varargin{1};
       if ~all(size(v) == [obj.nx,size(w,2)])
           error('Wrong dimension for the measurement errors "v"!'); 
       end
       w = [w;v];
    else
       v = zeros(obj.nx,size(w,2));
    end
    
    % simulate the system using different feedforward controllers
    if isa(obj.FeedforwardCtrl,'objGenSpaceContr')
        [t,x,u] = simulateGenSpace(obj,x0,w,v);
    else
        [t,x,u] = simulatePoly(obj,x0,w,v);
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end


% Auxiliary Functions -----------------------------------------------------

function [t,x,u] = simulateGenSpace(obj,x0,w,v)
% simulate controlled systems with generator space feed-forward controller    

    % time step size
    Nw = size(w,2)/obj.N;
    dt = obj.tFinal/(obj.N*Nw);
    
    % system dynamics
    if isempty(obj.V)
        f = @(t,x,w,p) extendedDynamics(x,w,p,obj.nx,obj.nu,...
                                   obj.dynamics,obj.linSys.A,obj.linSys.B);
    else
        f = @(t,x,w,p) extendedDynamicsMeasErr(x,w,p,obj.nx,obj.nu,...
                            obj.nw,obj.dynamics,obj.linSys.A,obj.linSys.B); 
    end
               
    % get linearization points
    Xc = obj.refTraj.xc;
    Uc = obj.refTraj.uc;
    Xlin = Xc(:,1:end-1) + 1/2*diff(Xc,1,2);
    Ulin = Uc;
    
    % initialization
    cx = center(obj.FeedforwardCtrl.parallelo{1});
    Gx = generators(obj.FeedforwardCtrl.parallelo{1});
    beta_ = Gx\(x0 + v(:,1) - cx);
    x_ = x0;
    xffc_ = cx;
    dxff_ = x0-cx;
    Gu_ff = generators(zonotope(obj.FeedforwardCtrl.U));
    
    % loop over all time steps
    x = []; t = []; u = []; counter = 1;
    
    for i = 1:obj.N
        
        % compute feed-forward control law
        alpha = obj.FeedforwardCtrl.alpha{1}{i};
        alpha_g = alpha(:,2:end);
        alpha_c = alpha(:,1);
        GuffAlpha = Gu_ff*alpha_g;
        uff_c = Gu_ff*alpha_c;
        
        % loop over all disturbances
        for j = 1:Nw
            
            % parameter vector and extended initial state
            p = [obj.K{i}(:);Xlin(:,i);Ulin(:,i);uff_c;GuffAlpha(:)];
            x_ext = [x_;xffc_;dxff_;beta_];
            
            if ~isempty(obj.V)
               x_ext = [x_ext; v(:,j)]; 
            end
            
            % simulate the system
            [tTemp,xTemp_ext] = ode45(@(t,x) f(t,x,w(:,counter),p),[0 dt],x_ext);
            
            % update initial state
            xTemp = xTemp_ext(:,1:obj.nx);
            xffc = xTemp_ext(:,obj.nx+1:2*obj.nx);
            dxff = xTemp_ext(:,2*obj.nx+1:3*obj.nx);
            beta = xTemp_ext(:,3*obj.nx+1:4*obj.nx);
            
            x_ = xTemp(end,:)'; xffc_ = xffc(end,:)'; 
            dxff_ = dxff(end,:)'; beta_ = beta(end,:)';
            counter = counter + 1;
            
            % reconstruct control input
            uff = uff_c + GuffAlpha*beta';
            xff = xffc + dxff;
            uTemp = (uff + obj.K{i}*(xTemp' - xff' + v(:,j)))';

            % store the results
            x = [x;xTemp]; 
            u = [u;uTemp];
            if isempty(t)
                t = [t;tTemp];
            else
                t = [t;tTemp + t(end)]; 
            end
        end
    end
end

function [t,x,u] = simulatePoly(obj,x0,w,v)
% simulate controlled systems with polynomial feed-forward controller
    
    % initialization
    cx = center(obj.FeedforwardCtrl.parallelo{1});
    x_ = x0;
    xffc_ = cx;
    dxff_ = x0-cx;
    
    % system dynamics
    f = @(t,x,w,p) extendedDynamicsPoly(x,w,p,obj.nx,obj.nu,obj.nw,...
                            obj.dynamics,obj.linSys.A,obj.linSys.B); 

    % feed-forward control law
    Uff = resolve(obj.FeedforwardCtrl.Ctrl_x{1},x0 + v(:,1));
    Uff = reshape(Uff,[obj.nu,obj.N]);

    % get linearization points
    Xc = obj.refTraj.xc;
    Uc = obj.refTraj.uc;
    Xlin = Xc(:,1:end-1) + 1/2*diff(Xc,1,2);
    Ulin = Uc;
    
    % time step size
    Nw = size(w,2)/obj.N;
    dt = obj.tFinal/(obj.N*Nw);
   
    % loop over all time steps
    x = []; t = []; u = []; counter = 1;
    
    for i = 1:obj.N
        
        % loop over all disturbances
        for j = 1:Nw

            % update parameter vector and extended initial state
            p = [obj.K{i}(:);Xlin(:,i);Ulin(:,i)];
            x_ext = [x_;xffc_;dxff_;Uff(:,i)];

            % simulate the system
            [tTemp,xTemp_ext] = ode45(@(t,x) f(t,x,w(:,counter),p),[0 dt],x_ext);

            % update initial state
            xTemp = xTemp_ext(:,1:obj.nx);
            xffc = xTemp_ext(:,obj.nx+1:2*obj.nx);
            dxff = xTemp_ext(:,2*obj.nx+1:3*obj.nx);

            x_ = xTemp(end,:)'; 
            xffc_ = xffc(end,:)'; 
            dxff_ = dxff(end,:)';

            % reconstruct control input
            uff = Uff(:,i);
            xff = xffc + dxff;
            uTemp = (uff + obj.K{i}*(xTemp' - xff' + v(:,counter)))';
            counter = counter + 1;

            % store the results
            x = [x;xTemp]; 
            u = [u;uTemp];
            if isempty(t)
                t = [t;tTemp];
            else
                t = [t;tTemp + t(end)]; 
            end
        end
    end
end