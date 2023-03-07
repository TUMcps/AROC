function [res,t,x,u] = simulate(obj,x0,w,varargin)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            the controller based on polynomial generator space control
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,x0,w)
%       [res,t,x,u] = SIMULATE(obj,x0,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the controller based polynomial control in generator space for 
%       a given initial point and given specific disturbance values over 
%       time.
%
% Input Arguments:
%
%       -obj:   object of class objPolyContr storing the control law
%               computed in the offline-phase
%       -x0:    initial point for the simulation
%       -w:     matrix storing the values for the disturbances over time
%               (dimension: [nw,N*Ninter])
%       -v:     matrix storing the values for the measurement errors over
%               time (dimension: [nx, N*Ninter])
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
%       objPolyContr, simulateRandom
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
% Authors:      Niklas Kochdumper,Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Arificial Intelligcen and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % check if the number of specified disturbance vectors is correct
    Nw = 1;
    
    if size(w,2) > 1
        Nw = size(w,2)/(obj.N * obj.Ninter);
        if Nw < 1 || floor(Nw) ~= Nw
           error('Number of disturbance vectors has to be a multiple of ''N*Ninter''!'); 
        end
    end

    % check if the number of specified measurement errors is correct
   	if nargin > 3
       v = varargin{1};
       if ~all(size(v) == [dim(obj.Rfin),obj.N])
           error('Wrong dimension for the measurement errors "v"!'); 
       end
    else
       v = zeros(obj.nx,obj.N);
    end

    % time step size
    dt = obj.tFinal/(obj.N*obj.Ninter*Nw);
    
    % dynamic function
    f = obj.dynamics;
    
    % initialization
    x_ = x0;
    counter = 1;
    nu  = length(center(obj.U));
    x = []; t = []; u = [];

    % loop over all time steps
    for i = 1:obj.N

        % get controller
        pZCtrl_i = obj.Ctrl_x{i};
        
        % compute current control law
        ui = resolve(pZCtrl_i,x_ + v(:,i)); 

        % loop over all intermediate time steps
        for j = 1:obj.Ninter
            
            u_ = ui((j-1)*nu+1:j*nu);
            
            % loop over all disturbances
            for k = 1:Nw
                
                % simulate the system
                [tTemp,xTemp] = ode45(@(t,x) f(x,u_,w(:,counter)),[0 dt],x_);
                
                % update initial state
                x_ = xTemp(end,:)';
                counter = counter + 1;
                
                % store the results
                x = [x;xTemp]; u = [u;ones(size(xTemp,1),1)*u_'];
                if isempty(t)
                    t = [t;tTemp];
                else
                    t = [t;tTemp + t(end)]; 
                end
            end
        end
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end