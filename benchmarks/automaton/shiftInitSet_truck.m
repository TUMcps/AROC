function Rinit = shiftInitSet_truck(Rinit,xf)
% SHIFTINITSET_TRUCK - shift initial set to final state of prev. maneuver
%
% Syntax:
%       Rinit = SHIFTINITSET_TRUCK(Rinit,xf)
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
%       truck, maneuverAuotomaton
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

    % shift the set to the origin of the position space
    c = center(Rinit);
    c([1,3,4]) = zeros(3,1);
    %c(3) = c(2);
    
    Rinit = zonotope(Rinit + (-c));
    
    % rotate the set by the orientation of the previous maneuver
    phi = xf(2);
    T = blkdiag(eye(4),[cos(phi) -sin(phi); sin(phi) cos(phi)]);
    Tinv = blkdiag(eye(4),[cos(c(2)) -sin(c(2)); sin(c(2)) cos(c(2))])';
    
    Rinit = T * Tinv * Rinit;
    
    % shift the set to the final state of the previous maneuver
    xf([1,3,4]) = zeros(3,1);
    %xf(3) = xf(2);
    Rinit = Rinit + xf;
end