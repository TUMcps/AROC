function im = animateQuadrotor2D(x,t,varargin)
% ANIMATEQUADROTOR2D - animate the simulated trajectories
%
% Syntax:
%       ANIMATEQUADROTOR2D(x,t)
%       ANIMATEQUADROTOR2D(x,t,statObs,dynObs,goalSet)
%
% Description:
%       Shows an animation that visualizes the given trajectory for the 
%       2D quadrotor benchmark together with static and dynamic obstacles.
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

        % plot quadrotor
        plotQuadrotor(x(i,:)');
        
        % formatting
        axis equal; box on;
        xticks([]); yticks([]);

        xlim([min(x(:,1))-0.5,max(x(:,1))+0.5]);
        ylim([min(x(:,2))-0.5,max(x(:,2))+0.5]);
        
        if i < size(x,1)
            if nargout > 0
                I = getframe(gca); im{end+1} = I.cdata;
            end
            pause(t(i+1)-t(i));
            clf; hold off;
        end
    end
end

function plotQuadrotor(x)
% visualize a 2D quadrocopter at the given position and orientation

    sx = x(1); sz = x(2); phi = x(3);

    % get quadrotopter in the local coordinate frame
    list = quadrocopter();
    
    % transform to global coordinate frame
    R = [cos(phi) sin(phi); -sin(phi) cos(phi)];
    
    for i = 1:length(list)
       list{i} = [sx;sz] + R*list{i};
    end
    
    % plot quadrocopter
    for i = 1:length(list)
        points = list{i};
        fill(points(1,:),points(2,:),'k'); 
    end
end

function list = quadrocopter()
% overall quadrotoper in the local coordinate frame

    % move the two rotors to the correct positions
    list1 = rotor();
    list2 = list1;
    
    for i = 1:length(list1)
        list1{i} = list1{i} + [-397;0];
    end
    
    for i = 1:length(list2)
        points = list2{i};
        points(1,:) = -points(1,:);
        list2{i} = points + [397;0];
    end
    
    % add the main body of the quadrocoper
    list3 = body();
    
    list = [list1,list2,list3];
    
    % scale quadrocopter to correct size
    for i = 1:length(list)
       list{i} = 1e-4 * list{i} - [0; 0.0145];
    end
end

function list = body()
% main body of the quadrocopter

    % connecting link
    x = [-390, -390, 0, 0];
    y = [212, 165, 165, 212];
    
    list{1} = [x;y];
    list{2} = [-x;y];
    
    % upper part
    x = [0, -68, -125, -125, 0];
    y = [292, 292, 247, 165, 165];
    
    list{3} = [x;y];
    list{4} = [-x;y];
    
    % lower part
    x = [0, -270, -76, 0];
    y = [165, 165, 125, 125];
    
    list{5} = [x;y];
    list{6} = [-x;y];
end

function list = rotor()
% a single rotor of the quadrocopter

    % rotor holder
    x = [0, -10, -13, -13, -21, -26, -26, -53, -53, 0];
    y = [293, 293, 289, 262, 262, 257, 125, 125, 76, 76];
    
    list{1} = [x;y];
    list{2} = [-x;y];
    
    % rotor blades
    points = rotorBlade();
    list{3} = points;
    points(1,:) = -points(1,:);
    list{4} = points;

    % foot
    x = [-19, -19, -53, -53];
    y = [125, 0, 0, 125];
    
    list{5} = [x;y];
end

function points = rotorBlade()
% a single rotor blade of the quadrocopter

    x = [-20.6, -20.6, -51, -226.72, -230, -230, -226.72];
    y = [287, 273, 258, 281.97, 284.09, 287.1, 289.89];

    points = [x;y];
end