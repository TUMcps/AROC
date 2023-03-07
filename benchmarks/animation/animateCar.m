function im = animateCar(x,t,varargin)
% ANIMATECAR - animate the simulated trajectories
%
% Syntax:
%       ANIMATECAR(x,t)
%       ANIMATECAR(x,t,statObs,dynObs,goalSet)
%       ANIMATECAR(x,t,statObs,dynObs,goalSet,lanelets)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       car benchmark together with static and dynamic obstacles.
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

        % plot car
        plotCar(x(i,:)');

        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,3))-5,max(x(:,3))+5]);
        ylim([min(x(:,4))-5,max(x(:,4))+5]);
        
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

function plotCar(x)
% plot the car

    % orientation and position of the car
    phi = x(2);
    x = x(3:4);

    T = [cos(phi) -sin(phi); sin(phi) cos(phi)];

    % construct polygons that define the car
    car = polygonCar();

    % transform car to the current position
    car = x + T*car;

    % plot the car
    plot(car,[1,2],'FaceColor',[42 127 255]/255);
end

function car = polygonCar()
% construct polygons that represent the car

    % car length and width in [m]
    l = 4.298;
    w = 1.674;
    
    % polygon for the car
    car = polygon([-l/2 -l/2 l/2 l/2],[w/2 -w/2 -w/2 w/2]);
end