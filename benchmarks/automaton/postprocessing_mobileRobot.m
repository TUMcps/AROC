function R = postprocessing_mobileRobot(R)
% POSTPROCESSING_MOBILEROBOT - compute the occupancy set from the reachable
%                              set for the mobile robot benchmarks
%
% Syntax:
%       R = POSTPROCESSING_MOBILEROBOT(R)
%
% Description:
%       This function computes the set occupied by the mobile robot form
%       the computed reachable set. The occupancy set is later on used for 
%       collision checking during online application of the Manuever 
%       Automaton. For the mobile robot benchmark, the reachable set is 
%       projected to the position states and bloated by the size of the
%       mobile robot.
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
%       mobileRobot, maneuverAuotomaton
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
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % robot length and width in [m]
    l = 0.455;
    w = 0.381;
    
    robot = zonotope(interval([-l/2;-w/2],[l/2;w/2]));
    
     % define function to compute occupancy set
    f = @(x,p) [x(1) + cos(x(3))*p(1) - sin(x(3))*p(2);
                x(2) + cos(x(3))*p(2) + sin(x(3))*p(1)];

    % compute occupancy set
    dt = 0.5;
    order = 3;
    tol = 0.01;

    R = postprocessingPolygon(f,robot,R,dt,order,tol);

end