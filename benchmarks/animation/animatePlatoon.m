function im = animatePlatoon(x,t)
% ANIMATEPLATOON - animate the simulated trajectories
%
% Syntax:
%       ANIMATEPLATOON(x,t)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       platoon benchmark.
%
% Input Arguments:
%
%       -x:             states of the trajectory (dimension: [N,nx])
%       -t:             times for the trajectory (dimension: [N,1])
%
% See Also:
%       animate
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

    w = warning();
    warning('off');
    im = [];

    % interpolate data
    N = ceil((t(end)-t(1))/0.05);
    t_ = linspace(t(1),t(end),N);
    
    [~,ind] = unique(t);

    x = interp1(t(ind),x(ind,:),t_);
    t = t_;

    % paramter
    c_safe = 20;        % minimum safe distance

    x_ref = min(x(:,1));
    x_min = x_ref - max(x(:,1)) - 10;
    x_max = x_ref + max(x(:,3)+x(:,5)+x(:,7))+4*c_safe+5;

    % loop over all time steps
    for i = 1:size(x,1)

        hold on;

        % plot lanes 
        lane1 = polygon([x_min-5,x_min-5,x_max+5,x_max+5],[-2,2,2,-2]);
        lane2 = polygon([x_min-5,x_min-5,x_max+5,x_max+5],[-6,-2,-2,-6]);

        plot(lane1,[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
        plot(lane2,[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');

        % plot platoon
        plotPlatoon(x(i,:)',c_safe,x_ref);

        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([x_min,x_max]);
        ylim([-10,6]);
        
        if i < size(x,1)
            if nargout > 0
                I = getframe(gca); im{end+1} = I.cdata;
            end
            pause(t(i+1)-t(i));
            clf; hold off;
        end
    end

    warning(w);
end

function plotPlatoon(x,c_safe,x_ref)
% plot the platoon
    
    % get polygons for the truck
    [truck,trailer] = polygonPlatoon();

    % loop over all vehicles
    offset = x_ref - x(1);

    for i = 1:4
        plot(truck + [offset;0],[1,2],'FaceColor',[42 127 255]/255);
        plot(trailer + [offset;0],[1,2],'FaceColor','w','EdgeColor','k');
        if i < 4
            offset = offset + c_safe + x(2*i+1);
        end
    end
end

function [truck,trailer] = polygonPlatoon()
% construct polygons that represent the car

    % truck length and width in [m]
    l = 5.1;             % length of the truck
    w = 2.55;            % width of the truck
    l_wb = 3.6;          % wheelbase of the truck
    l_t = 13.6;          % lenght of the trailer
    l_hitch = 12;        % hitch length trailer
    
    % polygons for the car
    truck = polygon([-l/2 -l/2 l/2 l/2],[w/2 -w/2 -w/2 w/2]);
    trailer = polygon([0 0 l_t l_t],[w/2 -w/2 -w/2 w/2]) + ...
                                    [l_t - l_hitch + l_wb - l; 0];
end