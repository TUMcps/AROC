function animateCommonRoad(obj,ind,dynObs,x0,goalSet,lanelets)
% ANIMATECOMMONROAD - show an animation of the planned trajectory
%
% Syntax:
%       ANIMATECOMMONROAD(obj,ind,dynObs,x0,goalSet,lanelets)
%
% Description:
%       This function shows an animation of a planned trajectory and 
%       displays the occupancy sets for the system as well as the dynamic 
%       obstacles for a CommonRoad traffic scenario
%
% Input Arguments:
%
%       -obj:       object of class maneuverAutomaton
%       -ind:       vector storing the indices of the motion-primitives for
%                   the planned trajectory
%       -dynObs:    cell-array storing the dynamic obstacles for the
%                   CommonRoad traffic scenario
%       -x0:        initial state for the CommonRoad traffic scenario
%       -goalSet:   goal set for the CommonRoad traffic scenario
%       -lanelets:  cell-array storing the lanelets for the CommonRoad
%                   traffic scenario
%
% See Also:
%       maneuverAuotomaton
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
    
    % compute the overall occupancy set for the planned trajectory
    x_pre = x0;
    time = 0;
    occSetAll = cell(length(ind),1);

    for i = 1:length(ind)
        x = obj.updateState(x_pre,ind(i)); 
        occSet = updateOccupancy(obj,x_pre,ind(i),time);
        occSetAll{i} = occSet;
        time = supremum(occSet{end}.time);
        x_pre = x;
    end

    % automatically determined bounds for figure
    bounds = [];
    for i = 1:length(lanelets)
       bounds = bounds | interval(lanelets{i}); 
    end
    
    % animation: loop over all time steps 
    for t = 0:0.1:(supremum(goalSet.time)-0.1)
        
        hold on; box on;
        timeInt = interval(t,t+0.1);
        
        % plot lanelets
        for i = 1:length(lanelets)
            plot(lanelets{i},[1,2],'FaceColor',[.7 .7 .7], ...
                 'Filled',true);
        end
        
        % set bounds for figures
        xlim([infimum(bounds(1)),supremum(bounds(1))]);
        ylim([infimum(bounds(2)),supremum(bounds(2))]);
        axis equal;

        % plot goal set and initial state
        plot(goalSet.set,[1,2],'r','EdgeColor','none', ...
             'FaceAlpha',0.5,'Filled',true);
        plot(x0(3),x0(4),'.g','MarkerSize',20);

        % plot the occupancy set
        for i = 1:length(occSetAll)
            for j = 1:length(occSetAll{i})
                if isIntersecting(occSetAll{i}{j}.time,timeInt)
                    plot(occSetAll{i}{j}.set,[1,2],'g','Filled',true, ...
                         'EdgeColor','none');
                end
            end
        end
        
        % plot the dynamic obstacles
        for d = 1:length(dynObs)
            if isIntersecting(dynObs{d}.time,timeInt)
                plot(dynObs{d}.set,[1,2],'b','Filled',true, ...
                     'EdgeColor','none');
            end
        end
        pause(0.1);
        hold off
    end
end