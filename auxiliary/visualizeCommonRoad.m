function visualizeCommonRoad(obj,ind,dynObs,x0,lanelets,time)
% VISUALIZECOMMONROAD - visualize a planned trajectory for different times
%
% Syntax:
%       VISUALIZECOMMONROAD(obj,ind,dynObs,x0,lanelets,time)
%
% Description:
%       This function visualizes the planned trajectory for a CommonRoad 
%       traffic scenario for the given time interval.
%
% Input Arguments:
%
%       -obj:       object of class maneuverAutomaton
%       -ind:       vector storing the indices of the motion-primitives for
%                   the planned trajectory
%       -dynObs:    cell-array storing the dynamic obstacles for the
%                   CommonRoad traffic scenario
%       -x0:        initial state for the CommonRoad traffic scenario
%       -lanelets:  cell-array storing the lanelets for the CommonRoad
%                   traffic scenario
%       -time:      time interval for which the trajectory is plotted
%                   (class: interval)
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
% Authors:      Jinyue Guan, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % plot road
    bounds = [];
    hold on;

    for i = 1:length(lanelets)
        plot(lanelets{i},[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
        bounds = bounds | interval(lanelets{i});
    end

    % plot the planned trajectory
    x_pre = x0;
    t = 0;

    for i = 1:length(ind)
        current_motion = ind(i);
        x = obj.updateState(x_pre,current_motion); 
        occSet = updateOccupancy(obj,x_pre,current_motion,t);
        for k = 1:length(occSet)
            if isIntersecting(time,occSet{k}.time)
                plot(occSet{k}.set,[1,2],'r','Filled',true, ...
                     'EdgeColor','none');
            else
                plot(occSet{k}.set,[1,2],'g','Filled',true, ...
                    'EdgeColor','none');
            end
        end
        t = supremum(occSet{end}.time);
        x_pre = x;
    end

    % plot occupancy sets of the other vehicles
    for i = 1:length(dynObs)
        if isIntersecting(time,dynObs{i}.time)
           plot(dynObs{i}.set,[1,2],'b','EdgeColor','none', ...
                'Filled',true); 
        end
    end

    % formatting
    xlim([infimum(bounds(1)),supremum(bounds(1))]);
    ylim([infimum(bounds(2)),supremum(bounds(2))]);
    axis equal, box on;
end