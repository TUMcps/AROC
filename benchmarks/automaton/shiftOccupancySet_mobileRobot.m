function R = shiftOccupancySet_mobileRobot(R,xf,time)
% SHIFTOCCUPANCYSET_MOBILEROBOT - shift occup. set to final state of 
%                                 prev. maneuver for the mobile robot
%
% Syntax:
%       R = SHIFTOCCUPANCYSET_MOBILEROBOT(R,xf,time)
%
% Description:
%       This function shifts the occupancy set of a maneuver to the final
%       state of the maneuver the was previously applied. This is required
%       for collision checking with dynamical obstacles during the online
%       execution of a maneuver automaton.
%
% Input Arguments:
%
%       -R:         cell-array storing the occupancy set
%       -xf:        final state of the previous maneuver
%       -time:      time at the end of the previous maneuver
%
% Output Arguments:
%
%       -R:         cell-array storing the shifted occupancy set
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

    % compute tranformation matrix and offset
    phi = xf(3);
    T = [cos(phi) -sin(phi); sin(phi) cos(phi)];
    o = xf(1:2);
    
    % loop over all reachable sets
    for i = 1:length(R)
        % transform the set
        R{i}.set = T*R{i}.set + o;
        R{i}.time = R{i}.time + time;
    end
end