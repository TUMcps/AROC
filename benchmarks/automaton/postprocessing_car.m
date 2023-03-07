function R = postprocessing_car(R)
% POSTPROCESSING_CAR - compute the occupancy set from the reachable set
%
% Syntax:
%       R = POSTPROCESSING_CAR(R)
%
% Description:
%       This function computes the set occupied by the car form the 
%       computed reachable set. The occupancy set is later on used for 
%       collision checking during online application of the Manuever 
%       Automaton. For the car benchmark, the reachable set is projected to 
%       the position states and bloated by the size of the car.
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
%       car, maneuverAuotomaton
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

    % car length and width in [m]
    l = 4.298;
    w = 1.674;
    
    car = zonotope(interval([-l/2;-w/2],[l/2;w/2]));
    
    % define function to compute occupancy set
    f = @(x,p) [x(3) + cos(x(2))*p(1) - sin(x(2))*p(2);
                x(4) + cos(x(2))*p(2) + sin(x(2))*p(1)];

    % compute occupancy set
    dt = 0.05;
    order = 3;
    tol = 0.002;

    R = postprocessingPolygon(f,car,R,dt,order,tol);
end