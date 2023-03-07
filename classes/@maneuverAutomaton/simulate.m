function [res,t,x,u] = simulate(obj,ind,x0,varargin)
% SIMULATE - simulate a trajectory of a nonlinear system controlled with 
%            a maneuver automaton
%
% Syntax:
%       [res,t,x,u] = SIMULATE(obj,ind,x0)
%       [res,t,x,u] = SIMULATE(obj,ind,x0,w)
%       [res,t,x,u] = SIMULATE(obj,ind,x0,w,v)
%
% Description:
%       Simulate a trajectory of a nonlinear closed-loop system controlled
%       with a maneuver automaton for a given initial point and given 
%       specific disturbance values over time.
%
% Input Arguments:
%
%       -obj:   object of class maneuverAutomaton storing the maneuver
%               automaton
%       -ind:   indices of the motion primitives for the planned trajectory
%       -x0:    initial point for the simulation
%       -w:     cell-array storing the values for the disturbances for the
%               single motion primitives
%       -v:     cell-array storing the values for the measurement errors
%               for the single motion primitives
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
%       maneuverAutomaton, simulateRandom
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

    % check if the number of specified disturbance vectors is correct
    if nargin > 2
        w = varargin{1};
        if iscell(w)
            if length(w) ~= length(ind)
                error(['Number of disturbances must match the ', ...
                       'number of motion primitives!']);
            end
        elseif isnumeric(w)
            w = repmat({w},[length(ind),1]);
        else
            error('Wrong format for disturbances w!');
        end
    else
        w = center(obj.motionPrimitives{1},W);
        w = repmat({w},[length(ind),1]);
    end
    
    % check if the number of specified measurement errors is correct
   	if nargin > 3
        v = varargin{2};
        if iscell(v)
            if length(v) ~= length(ind)
                error(['Number of measurement errors must match the ', ...
                       'number of motion primitives!']);
            end
        elseif isnumeric(v)
            v = repmat({v},[length(ind),1]);
        else
            error('Wrong format for measurement errors v!');
        end
    else
       v = zeros(obj.primitives{1}.nx,length(ind));
       v = repmat({v},[length(ind),1]);
    end

    % initialization
    x_ = x0; t = []; x = []; u = [];

    % loop over all motion primitives
    for i = 1:length(ind)
    
        % get initial state in the coordinate frame of the motion primitive
        R0 = zonotope(interval(obj.primitives{ind(i)}.R0));
        R0_ = obj.shiftFun(R0,x_);

        alpha = generators(R0_) \ (x0 - center(R0_));
        x0 = center(R0) + generators(R0) * alpha;

        % simulate controller for the current motion primitive
        [~,tTmp,xTmp,uTmp] = simulate(obj.primitives{ind(i)},x0,w{i},v{i});

        % transform trajectories to global coordinate system
        for j = 1:size(xTmp,1)
            c = center(R0);
            tmp = obj.shiftFun(zonotope(c,xTmp(j,:)'-c),x_);
            xTmp(j,:) = (center(tmp) + generators(tmp))';
        end

        x0 = xTmp(end,:)';

        % store simulation results
        if isempty(t)
            t = [t;tTmp];
        else
            t = [t;tTmp + t(end)]; 
        end

        x = [x; xTmp]; u = [u; uTmp];

        % update state
        x_ = obj.updateState(x_,ind(i));
    end

    % store simulation in results object
    sim{1}.t = t;
    sim{1}.x = x;
    sim{1}.u = u;
    res = results([],[],[],sim);
end