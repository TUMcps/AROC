function im = animateMobileRobot(x,t,varargin)
% ANIMATEMOBILEROBOT - animate the simulated trajectories
%
% Syntax:
%       ANIMATEMOBILEROBOT(x,t)
%       ANIMATEMOBILEROBOT(x,t,statObs,dynObs,goalSet)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       mobile robot benchmark together with static and dynamic obstacles.
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
    statObs = []; dynObs = []; goalSet = []; im = [];

    if nargin > 2 && ~isempty(varargin{1})
        statObs = varargin{1};
    end

    if nargin > 3 && ~isempty(varargin{2})
        dynObs = varargin{2};
    end

    if nargin > 4 && ~isempty(varargin{3})
        goalSet = varargin{3};
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

        % plot goal set
        if ~isempty(goalSet)
            plot(goalSet,[1,2],'FaceColor',[134 227 134]/255);
        end

        % plot obstacles
        if ~isempty(statObs) || ~isempty(dynObs)
            plotObstacles(statObs,dynObs,interval(t(i)-0.05,t(i)+0.05));
        end

        % plot mobile robot
        plotMobileRobot(x(i,:)');

        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,1))-0.6,max(x(:,1))+0.6]);
        ylim([min(x(:,2))-0.6,max(x(:,2))+0.6]);
        
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

function plotMobileRobot(x)
% plot the mobile robot

    % orientation and position of the mobile robot
    phi = x(3);
    x = x(1:2);

    T = [cos(phi) -sin(phi); sin(phi) cos(phi)];

    % construct polygons that define the mobile robot
    [body,wheels] = mobileRobot();

    % transform mobile robot to the current position
    body = x + T*body;
    wheels = x + T*wheels;

    % plot the mobile robot
    plot(body,[1,2],'FaceColor',[42 127 255]/255);
    plot(wheels,[1,2],'FaceColor',[.7 .7 .7]);
end

function [body,wheels] = mobileRobot()
% construct polygons that represent the mobile robot 

    % robot length and width
    l = 0.455;
    w = 0.381;
    
    % polygon for the robot body
    phi = linspace(0,2*pi,100);

    body = polyshape(l/2*cos(phi),l/2*sin(phi));
    body = subtract(body,polyshape([-l -l l l]/2,[2*w w w 2*w]/2));
    body = subtract(body,polyshape([-l -l l l]/2,-[2*w w w 2*w]/2));
    body = subtract(body,polyshape([-l -l -0.3*l -0.05*l]/2, ...
                                         [1.1*w 0.85*w 0.85*w 1.1*w]/2));
    body = subtract(body,polyshape([-l -l -0.3*l -0.05*l]/2, ...
                                        -[1.1*w 0.85*w 0.85*w 1.1*w]/2));

    % polygon for the wheels
    wheels = polyshape([-0.01 -0.01 0.1 0.1],[0.9*w 0.75*w 0.75*w 0.9*w]/2);
    wheels = union(wheels,polyshape([-0.01 -0.01 0.1 0.1], ...
                                        -[0.9*w 0.75*w 0.75*w 0.9*w]/2));
    wheels = union(wheels,polyshape([-0.9*l -0.9*l -0.6*l -0.6*l]/2, ...
                                    [0.05*w -0.05*w -0.05*w 0.05*w]/2));

    % convert to polygon objects
    V = body.Vertices;
    body = polygon(V(:,1),V(:,2));

    V = wheels.Vertices;
    wheels = polygon(V(:,1),V(:,2));
end