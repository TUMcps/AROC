function R = postprocessing_robotArm(R)
% POSTPROCESSING_ROBOTARM - comp. the occupancy set from the reachable set
%
% Syntax:
%       R = POSTPROCESSING_ROBOTARM(R)
%
% Description:
%       This function computes the set occupied by the robot arm from the 
%       computed reachable set. The occupancy set is later on used for 
%       collision checking during online application of the Manuever 
%       Automaton. For the robot arm benchmark, the reachable set is
%       converted from joint space into the Euclidean space.
%
% Input Arguments:
%
%       -R:     cell-array storing the reachable set
%
% Output Arguments:
%
%       -R:     cell-array storing the transformed reachable set
%
% See Also:
%       robotArm, maneuverAuotomaton
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

    % parameter
    l1 = 0.2;                   % length of the first link
    l2 = 0.2;                   % length of the second link
    w = 0.06;                   % width of the links
    
    % settings for occupancy set computation
    dt = 0.05;
    order = 3;
    tol = 0.05;

    % compute occupancy set for the first link
    link1 = zonotope(interval([0;-w/2],[l1;w/2]));
    
    f = @(x,p) [cos(x(1))*p(1) - sin(x(1))*p(2);
                cos(x(1))*p(2) + sin(x(1))*p(1)];

    R1 = postprocessingPolygon(f,link1,R,dt,order,tol);

    % compute occupancy set for the second link
    link2 = zonotope(interval([0;-w/2],[l2;w/2]));
    
    f = @(x,p) [l1*cos(x(1)) + cos(x(1)+x(2))*p(1) - sin(x(1)+x(2))*p(2);
                l1*sin(x(1)) + cos(x(1)+x(2))*p(2) + sin(x(1)+x(2))*p(1)];

    R2 = postprocessingPolygon(f,link2,R,dt,order,tol);

    % compute occupancy sets for truck and trailer
    R = R1;

    for i = 1:length(R)
        R{i}.set = R1{i}.set | R2{i}.set;
    end
end