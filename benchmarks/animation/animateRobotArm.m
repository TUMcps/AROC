function im = animateRobotArm(x,t,varargin)
% ANIMATEROBOTARM - animate the simulated trajectories
%
% Syntax:
%       ANIMATEROBOTARM(x,t)
%       ANIMATEROBOTARM(x,t,statObs,dynObs,goalSet)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       robot arm benchmark.
%
% Input Arguments:
%
%       -x:         states of the trajectory (dimension: [N,nx])
%       -t:         times for the trajectory (dimension: [N,1])
%       -statObs:   cell-array storing the static obstacles
%       -dynObs:    cell-array storing the dynamic obstacles. Each
%                   dynamic obstacle is represented as a struct with 
%                   fields .set and .time 
%       -goalSet:   goal set for the motion planning problem
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

        % plot robot arm
        plotRobotArm(x(i,:)');
        
        % formatting 
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([-0.5,0.5]);
        ylim([-0.5,0.5]);

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

function plotRobotArm(x)
% plot the cartpole at the current position

    % robot arm parameters
    l1 = 0.2;
    l2 = 0.2;
    w = 0.03;

    % links 
    link1 = polygon([0 0 l1 l1],[-w/2 w/2 w/2 -w/2]);
    link2 = polygon([0 0 l2-0.06 l2-0.06],[-w/2 w/2 w/2 -w/2]);

    % joints
    phi = 0:0.01:2*pi;
    joint = polygon(0.02*cos(phi),0.02*sin(phi));

    % base
    base1 = polygon([-0.04 0.04 0.03 -0.03],[-0.03 -0.03 0.03 0.03]);
    base2 = polygon([-0.06 0.06 0.06 -0.06],[-0.05 -0.05 -0.03 -0.03]);

    % gripper
    gripper1 = [l2-0.06;0] + polygon([0 0 0.03 0.06 0.06 0.03], ...
                                     [0.01 0.02 0.03 0.02 0.01 0.02]);
    gripper2 = [1 0;0 -1]*gripper1;
    gripper3 = [l2-0.06;0] + polygon([-0.005 -0.005 0.005 0.005], ...
                                     [-0.025 0.025 0.025 -0.025]);

    % rotation matrices
    R1 = [cos(x(1)) -sin(x(1)); sin(x(1)) cos(x(1))];
    R2 = [cos(x(1)+x(2)) -sin(x(1)+x(2)); sin(x(1)+x(2)) cos(x(1)+x(2))];

    % transform links and gripper to the correct positions
    link1 = R1*link1;
    link2 = R2*link2 + R1*[l1;0];
    gripper1 = R2*gripper1 + R1*[l1;0];
    gripper2 = R2*gripper2 + R1*[l1;0];
    gripper3 = R2*gripper3 + R1*[l1;0];

    % plot robot
    plot(base1,[1,2],'FaceColor',[.5 .5 .5]);
    plot(base2,[1,2],'FaceColor',[.5 .5 .5]);
    plot(link1,[1,2],'FaceColor',[0 102 255]/255);
    plot(link2,[1,2],'FaceColor',[0 102 255]/255);
    plot(joint,[1,2],'FaceColor','w','EdgeColor','k','LineWidth',1);
    plot(joint+R1*[l1;0],[1,2],'FaceColor','w', ...
                        'EdgeColor','k','LineWidth',1);
    plot(gripper1,[1,2],'FaceColor','k');
    plot(gripper2,[1,2],'FaceColor','k');
    plot(gripper3,[1,2],'FaceColor','k');
end