function plotPlannedTrajectory(obj,ind,x0,varargin)
% PLOTPLANNEDTRAJECTORY - plot occupancy set for the planned trajectory
%
% Syntax:
%       PLOTPLANNEDTRAJECTORY(obj,ind,x0)
%       PLOTPLANNEDTRAJECTORY(obj,ind,x0,time)
%       PLOTPLANNEDTRAJECTORY(obj,ind,x0,time,color,options)
%
% Description:
%       This function visualizes the occupancy set for a trajectory planned 
%       with a maneuver automaton. By default the whole trajectory is
%       visualized, but one can also only visualize the trajectory for a
%       specific time interval.
%
% Input Arguments:
%
%       -obj:       object of class maneuverAutomaton
%       -ind:       vector storing the indices of the motion-primitives for
%                   the planned trajectory
%       -x0:        initial state for the trajectory
%       -time:      time interval for which the trajectory is plotted
%                   (class: interval)
%       -color:     color for the plot specified as a string (e.g. 'r') or
%                   as a vector of RGB-values (e.g. [0 1 0])
%       -options:   additional MATLAB plot options specified as name-value
%                   pairs (e.g. 'LineWidth',2)
%
% See Also:
%       maneuverAutomaton
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

    % default values
    time = [];
    color = [.6 .6 .6];
    NVpairs = {};

    % parse input arguments  
    if nargin > 3 && ~isempty(varargin{1})
        time = varargin{1};
    end

    if nargin > 4 && ~isempty(varargin{2})
       color = varargin{2};
       if ~ischar(color) && (any(size(color) ~= [3,1]) && ...
          any(size(color) ~= [1,3]))
           error('Wrong value for input argument "color"! Has to be a string or a 3D vector of RGB values!') 
       end
    end
    
    if nargin > 5
       NVpairs = varargin(3:end); 
       color = getNameValuePair(NVpairs,'FaceColor',color);
    end

    % plot the planned trajectory
    x_pre = x0;
    t = 0;
    hold on;

    for i = 1:length(ind)

        % update occupancy set for the current motion primitive
        current_motion = ind(i);
        x = obj.updateState(x_pre,current_motion); 
        occSet = updateOccupancy(obj,x_pre,current_motion,t);

        % plot overall occupancy set
        if isempty(time)

            dims = 1:dim(occSet{1}.set);

            if length(dims) == 2
                poly = [];
                for k = 1:length(occSet)
                    poly = poly | occSet{k}.set;    
                end
                plot(poly,[1,2],'FaceColor',color,NVpairs{:});
            else
                for k = 1:length(occSet)
                    plot(occSet{k}.set,dims,'FaceColor',color,NVpairs{:});
                end
            end

        % plot occupancy set at a specific time
        else
            
            for k = 1:length(occSet)
                dims = 1:dim(occSet{1}.set);
                if isIntersecting(time,occSet{k}.time)
                    plot(occSet{k}.set,dims,'FaceColor',color,NVpairs{:});
                end
            end
        end

        t = supremum(occSet{end}.time);
        x_pre = x;
    end