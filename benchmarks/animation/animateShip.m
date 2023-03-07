function im = animateShip(x,t,varargin)
% ANIMATESHIP - animate the simulated trajectories
%
% Syntax:
%       ANIMATESHIP(x,t)
%       ANIMATESHIP(x,t,statObs,dynObs,goalSet)
%       ANIMATESHIP(x,t,statObs,dynObs,goalSet,waters)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       ship benchmark together with static and dynamic obstacles.
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
%       -waters:        cell-array storing the water ways
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
    statObs = []; dynObs = []; goalSet = []; waters = []; im = [];

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
        waters = varargin{4};
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

        % plot background
        background = interval([min(x(:,1))-150;min(x(:,2)-150)], ...
                              [max(x(:,1))+150;max(x(:,2)+150)]);

        plot(background,[1,2],'FaceColor',[85 153 255]/255);

        % plot road
        if ~isempty(waters)
            for j = 1:length(waters)
                plot(waters{j},[1,2],'FaceColor','b','EdgeColor','k');
            end
        end

        % plot goal set
        if ~isempty(goalSet)
            plot(goalSet,[1,2],'FaceColor',[0 0.85 0]);
        end

        % plot obstacles
        if ~isempty(statObs) || ~isempty(dynObs)
            plotObstacles(statObs,dynObs,interval(t(i)-0.05,t(i)+0.05));
        end

        % plot ship
        plotShip(x(i,:)');

        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,1))-100,max(x(:,1))+100]);
        ylim([min(x(:,2))-100,max(x(:,2))+100]);
        
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

function plotShip(x)
% plot the ship

    % orientation and position of the car
    phi = x(3);
    x = x(1:2);

    T = [cos(phi) -sin(phi); sin(phi) cos(phi)];

    % construct polygons that define the car
    ship = polygonsShip();

    % transform car to the current position
    for i = 1:length(ship)
        ship{i} = x + T*ship{i};
    end

    % plot the ship
    plot(ship{1},[1,2],'FaceColor',[.5 .5 .5],'EdgeColor','k');
    plot(ship{2},[1,2],'FaceColor','w','EdgeColor','k');
    plot(ship{3},[1,2],'FaceColor','w','EdgeColor','k');
    plot(ship{4},[1,2],'FaceColor',[.2 .2 .2]);

    for i = 5:length(ship)
        if mod(i,4) == 0
            plot(ship{i},[1,2],'FaceColor',[0 0.7 0],'EdgeColor','k');
        elseif mod(i,3) == 2
            plot(ship{i},[1,2],'FaceColor',[42 127 255]/255,'EdgeColor','k');
        else
            plot(ship{i},[1,2],'FaceColor',[0.7 0 0],'EdgeColor','k');
        end
    end
end

function ship = polygonsShip()
% construct polygons that represent the ship

    % ship length and width in [m]
    l = 175;
    w = 25.4;
    
    % ship body
    phi = linspace(-pi/2,pi/2,100);
    bug = polygon(l/6*cos(phi),w/2*sin(phi)) + [2*l/6;0];
    ship{1} = polygon([-l/2 -l/2 -3*l/8 l/3 l/3 -3*l/8],...
                      [w/4 -w/4 -w/2 -w/2 w/2 w/2]) | bug;

    % bridge
    ship{2} = polygon([0 0 l l]/30,[w/2 -w/2 -w/2 w/2]*9/10) - [3.2/8*l;0];
    ship{3} = polygon([-13.5/30*l -13.5/30*l -3/8*l -3/8*l], ...
                      [w/5 -w/5 -w/5 w/5]) - [0.2/8*l;0]; 
    ship{4} = polygon([-13.3/30*l -13.3/30*l -3.2/8*l -3.2/8*l], ...
                      [w/10 -w/10 -w/10 w/10]) - [0.2/8*l;0]; 

    % containers
    I = interval([-2.8*l/8;-9/20*w],[l/3;9/20*w]);
    w = 2*rad(I);
    N_l = 10; N_w = 5;
    lc = w(1)/N_l; wc = w(2)/N_w;

    for i = 1:N_l
        for j = 1:N_w
            ship{end+1} = infimum(I) + interval([(i-1)*lc;(j-1)*wc],[i*lc;j*wc]);
        end
    end

end