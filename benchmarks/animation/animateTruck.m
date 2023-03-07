function im = animateTruck(x,t,varargin)
% ANIMATETRUCK - animate the simulated trajectories
%
% Syntax:
%       ANIMATETRUCK(x,t)
%       ANIMATETRUCK(x,t,statObs,dynObs,goalSet)
%       ANIMATETRUCK(x,t,statObs,dynObs,goalSet,lanelets)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       truck benchmark together with static and dynamic obstacles.
%
% Input Arguments:
%
%       -x:             states of the trajectory (dimension: [N,nx])
%       -t:             times for the trajectory (dimension: [N,1])
%       -statObs:       cell-array storing the static obstacles
%       -dynObs:        cell-array storing the dynamic obstacles. Each
%                       dynamic obstacle is represented as a struct with 
%                       fields .set and .time 
%       -goalSet:       goal set for the motion planning problem
%       -lanelets:      cell-array storing the lanelets of the road
%
% See Also:
%       animate
%
% References:
%       * *[1] Althoff et al. (2020)*, CommonRoad: Vehicle Models
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

    % parse input arguments
    statObs = []; dynObs = []; goalSet = []; lanelets = []; im = [];

    if nargin > 2 && ~isempty(varargin{1})
        statObs = varargin{1};
    end

    if nargin > 3 && ~isempty(varargin{2})
        dynObs = varargin{2};
    end

    if nargin > 4 && ~isempty(varargin{3})
        goalSet = varargin{3};
    end

    if nargin > 5 && ~isempty(varargin{4})
        lanelets = varargin{4};
    end

    w = warning();
    warning('off');

    % interpolate data
    N = ceil((t(end)-t(1))/0.05);
    t_ = linspace(t(1),t(end),N);
    
    [~,ind] = unique(t);

    x = interp1(t(ind),x(ind,:),t_);
    t = t_;

    % loop over all time steps
    for i = 1:size(x,1)

        hold on;

        % plot road
        if ~isempty(lanelets)
            for j = 1:length(lanelets)
                plot(lanelets{j},[1,2],'FaceColor',[.6 .6 .6],'EdgeColor','k');
            end
        end

        % plot goal set
        if ~isempty(goalSet)
            plot(goalSet,[1,2],'FaceColor',[134 227 134]/255,'EdgeColor','k');
        end

        % plot obstacles
        if ~isempty(statObs) || ~isempty(dynObs)
            plotObstacles(statObs,dynObs,interval(t(i)-0.05,t(i)+0.05));
        end

        % plot truck
        plotTruck(x(i,:)');

        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,5))-20,max(x(:,5))+20]);
        ylim([min(x(:,6))-20,max(x(:,6))+20]);
        
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

function plotTruck(x)
% plot the truck

    % orientation and position of truck and trailer
    phi_1 = x(2);
    phi_2 = x(2) + x(3);
    x = x(5:6);

    T_truck = [cos(phi_1) -sin(phi_1); sin(phi_1) cos(phi_1)];
    T_trailer = [cos(phi_2) -sin(phi_2); sin(phi_2) cos(phi_2)];

    % construct polygons that define the truck
    [truck,trailer] = truckAndTrailer();

    % transform truck to the current position
    truck = x + T_truck*truck;
    trailer = x + T_trailer*trailer;

    % plot the truck
    hold on;
    plot(truck,[1,2],'FaceColor',[42 127 255]/255);
    plot(trailer,[1,2],'FaceColor','w','EdgeColor','k');
end

function [truck,trailer] = truckAndTrailer()
% construct polygons that represent the truck and the trailer 

    % parameter (see Table 3 in [1])
    l = 5.1;             % length of the truck
    w = 2.55;            % width of the truck
    l_wb = 3.6;          % wheelbase of the truck
    l_t = 13.6;          % lenght of the trailer
    l_hitch = 12;        % hitch length trailer

    % polygon representing the truck
    l1 = l_wb + (l - l_wb)/2;
    l2 = (l - l_wb)/2;

    truck = polygon([-l2 -l2 l1 l1],[w/2 -w/2 -w/2 w/2]);

    % polygon representing the trailer
    dl = l_t - l_hitch;

    trailer = polygon([dl dl -l_hitch -l_hitch],[w/2 -w/2 -w/2 w/2]);
end