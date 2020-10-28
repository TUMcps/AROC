function Rinit = shiftInitSet_mobileRobot(Rinit,xf)
% SHIFTINITSET_MOBILEROBOT - shift initial set to the final state of 
%                            prev. maneuver for the mobile robot benchmark
%
% Syntax:
%       Rinit = SHIFTINITSET_MOBILEROBOT(Rinit,xf)
%
% Description:
%       This function shifts the initial set of a maneuver to the final
%       state of the maneuver the was previously applied. This function is
%       required to check which maneuvers can be concatenated in order to
%       solve online control tasks with a maneuver automaton.
%
% Input Arguments:
%
%       -Rinit:     inital set
%       -xf:        final state of the previous maneuver
%
% Output Arguments:
%
%       -Rinit:     shifted initial set
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

    % shift the set to the origin of the position space
    c = center(Rinit);
    c(4) = 0;
    c(5) = 0;
    
    Rinit = zonotope(Rinit + (-c));
    
    % rotate the set by the orientation of the previous maneuver
    phi = xf(3);
    T = blkdiag([cos(phi) -sin(phi); sin(phi) cos(phi)],eye(3));
    Tinv = blkdiag([cos(c(2)) -sin(c(2)); sin(c(2)) cos(c(2))],eye(3))';
    
    Rinit = T * Tinv * Rinit;
    
    % shift the set to the final state of the previous maneuver
    xf(4) = 0;
    xf(5) = 0;
    Rinit = Rinit + xf;
end