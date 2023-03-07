function [res,t,x,u] = simulate(obj,x0,tFinal,w,varargin)
% SIMULATE - simulate a trajectory of a terminal region controller
%           
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,x0,tFinal,w)
%       [res,t,x,u] = SIMULATE(obj,x0,tFinal,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with the terminal region controller corresponding to a terminal
%       region computed with the subpaving algorithm.
%
% Input Arguments:
%
%       -obj:      object of class termRegSubpaving storing the control law
%                  computed for the terminal region controller.
%       -x0:       initial point for the simulation
%       -tFinal:   final time for the simulation
%       -w:        matrix storing the values for the disturbances over time
%                  (dimension: [nw,timeSteps])
%       -v:        matrix storing the values for the measurement errors
%                  over time (dimension: [nx,timeSteps])
%
% Output Arguments:
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input 
%               (dimension: [|t|,nu])
%
% See Also:
%       compTermRegSubpaving, simulateRandom, termRegSubpaving
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % parse input arguments
    if nargin > 5 && ~isempty(varargin{1})
        v = varargin{1};
        if size(v,1) ~= dim(obj.set)
           error('Wrong dimension for the measurement errors "v"!');
        elseif size(v,2) ~= size(w,2)   
           error('Wrong number of measurements errors "v"!');
        end
    else
        v = zeros(dim(obj.set),size(w,2));
    end

    % compute time step 
    timeStep = tFinal/size(w,2);
    
    % simulate each part with constant input
    x_ = x0;
    x = []; u = []; t = [];
    
    for i = 1:size(w,2)
 
        % define closed-loop dynamics
        fun = @(x,w,v) obj.dynamics(x,obj.uEq + obj.K*(x+v-obj.xEq),w);
            
        % simulate the system
        [tTemp,xTemp] = ode45(@(t,x)fun(x,w(:,i),v(:,i)),[0 timeStep],x_);

        x_ = xTemp(end,:)';
        x = [x;xTemp];
        u = [u;(obj.uEq + obj.K*(xTemp' - obj.xEq))'];
        if isempty(t)
            t = [t;tTemp];
        else
            t = [t;t(end)+tTemp];
        end
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end