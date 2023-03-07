function R = shiftOccupancySet_robotArm(R,xf,time)
% SHIFTOCCUPANCYSET_ROBOTARM - shift set to final state of prev. maneuver
%
% Syntax:
%       R = SHIFTOCCUPANCYSET_ROBOTARM(R,xf,time)
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
%       robotArm, maneuverAutomaton
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

    % loop over all occupancy sets
    for i = 1:length(R)
        R{i}.time = R{i}.time + time;
    end
end