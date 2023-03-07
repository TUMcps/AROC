function R = postprocessing_truck(R)
% POSTPROCESSING_TRUCK - compute the occupancy set from the reachable set
%
% Syntax:
%       R = POSTPROCESSING_TRUCK(R)
%
% Description:
%       This function computes the set occupied by the truck from the 
%       computed reachable set. The occupancy set is later on used for 
%       collision checking during online application of the Manuever 
%       Automaton. For the truck benchmark, the reachable set is projected
%       to the position states and bloated by the size of the truck.
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
%       truck, maneuverAuotomaton
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

    % parameter (see Table 3 in [1])
    l = 5.1;             % length of the truck
    w = 2.55;            % width of the truck
    l_wb = 3.6;          % wheelbase of the truck
    l_t = 13.6;          % lenght of the trailer
    l_hitch = 12;        % hitch length trailer
    
    % settings for occupancy set computation
    dt = 0.05;
    order = 3;
    tol = 0.05;

    % compute occupancy set for the truck
    l1 = l_wb + (l - l_wb)/2;
    l2 = (l - l_wb)/2;

    truck = zonotope(interval([-l2;-w/2],[l1;w/2]));
    
    f = @(x,p) [x(5) + cos(x(2))*p(1) - sin(x(2))*p(2);
                x(6) + cos(x(2))*p(2) + sin(x(2))*p(1)];

    R1 = postprocessingPolygon(f,truck,R,dt,order,tol);

    % compute occupancy set for the trailer
    dl = l_t - l_hitch;

    trailer = zonotope(interval([-l_hitch;-w/2],[dl;w/2]));

    f = @(x,p) [x(5) + cos(x(2)+x(3))*p(1) - sin(x(2)+x(3))*p(2);
                x(6) + cos(x(2)+x(3))*p(2) + sin(x(2)+x(3))*p(1)];

    R2 = postprocessingPolygon(f,trailer,R,dt,order,tol);

    % compute occupancy sets for truck and trailer
    R = R1;

    for i = 1:length(R)
        R{i}.set = R1{i}.set | R2{i}.set;
    end
end