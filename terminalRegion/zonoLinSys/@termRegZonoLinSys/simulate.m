function [res,t,x,u] = simulate(obj,x0,tFinal,w,varargin)
% SIMULATE - simulate a trajectory of a terminal region controller
%           
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,x0,tFinal,w)
%       [res,t,x,u] = SIMULATE(obj,x0,tFinal,w,v)
%
% Description:
%       Simulate a trajectory of a linear closed-loop system controlled
%       with the terminal region controller corresponding to a terminal
%       region computed with the zonotope approach for linear systems.
%
% Input Arguments:
%
%       -obj:      object of class termRegZonoLinSys storing the control
%                  law computed for the terminal region controller.
%       -x0:       initial point for the simulation
%       -tFinal:   final time for the simulation
%       -w:        matrix storing the values for the disturbances over time
%                  (dimension: [nw,timeSteps])
%       -v:        matrix storing the values for the measurement errors 
%                  over time (dimension: [nv, tFinal/dt-1]) 
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -x:     matrix storing the simulated trajectory 
%               (dimension: [|t|,nx])
%       -u:     matrix storing the applied control input 
%               (dimension: [|t|,nu])
%
% See Also:
%       computeTermRegZonoLinSys, simulateRandom, termRegZonoLinSys
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
% Authors:      Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % initialization 
    x = []; u = []; t = [];
    nx = length(x0);
    tw = linspace(0,tFinal,size(w,2)+1);
    
    % parse input arguments
   	if nargin >= 5
       v = varargin{1};
       if size(v,1) ~= nx
           error('Wrong dimension for the measurement errors "v"!');
       elseif size(v,2) ~= floor(tFinal/obj.dt)   
           error('Wrong number of measurements errors "v"!');
       end
    else
       v = zeros(nx,floor(tFinal/obj.dt));
    end
    
    % extend input sets by zeros to implement the terminal controller
    inputSets = obj.inputSets;
    m = ceil((tFinal - length(inputSets)*obj.dt)/obj.dt);
    for i = 1:m
       inputSets = [inputSets, {0 * inputSets{1}}]; 
    end

    % determine zonotope factors for input correction
    alpha = getInputScalings(obj,x0);

    % loop over all time steps
    for i = 1:length(inputSets)

        % get disturbance signals for current time step
        ind = [];
        
        for j = 1:length(tw)-1
           int1 = interval((i-1)*obj.dt,i*obj.dt);
           int2 = interval(tw(j),tw(j+1));
           tmp = int1 & int2;
           if ~isempty(tmp) && rad(tmp) > 0
              ind = [ind, j]; 
           end
        end
        
        % compute control input
        c_u = center(inputSets{i});
        G_u = generators(inputSets{i});
        
        u = obj.uEq + obj.K*(x0 - obj.xEq + v(:,i)) + c_u + G_u*alpha;

        % loop over all disturbance signals
        for j = 1:length(ind)
        
            % simulate the system
            tStart = max((i-1)*obj.dt,tw(ind(j)));
            tFinal = min(i*obj.dt,tw(ind(j)+1));
            
            [tTemp,xTemp] = ode45(@(t,x)obj.dynamics(x,u,w(:,ind(j))), ...
                                                    [tStart tFinal],x0);

            % store simulation results
            x = [x;xTemp(:,1:nx)];
            u = [u;xTemp(:,1:nx+1:end)];
            t = [t;tTemp];
            x0_ = xTemp(end,:)';
        end
        
        x0 = x0_(1:nx);
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end


% Auxiliary Functions -----------------------------------------------------

function scalings = getInputScalings(obj,x0)
% compute scaling factors for the input zonotope

    % initialization
    center = obj.set.center;
    generators = obj.set.generators;
    [nx, nGen] = size(generators);
    
    % setup optimization problem
    state = sdpvar(nx, 1, 'full');
    scalings = sdpvar(nGen, 1, 'full');
    constraings = [state == center + generators * scalings];
    cost = norm(scalings, 'inf');
    options = sdpsettings('verbose',0,'allownonconvex',0, ...
                                                    'solver','linprog');
    opt = optimizer(constraings, cost, options, state, scalings);
    
    % compute input scaling by solving the optimization problem
    scalings = opt(x0);
end