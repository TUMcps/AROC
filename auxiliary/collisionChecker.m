function res = collisionChecker(occSet,statObs,dynObs)
% COLLISIONCHECKER - check if the occupancy set collides with obstacles
%
% Syntax:
%       res = COLLISIONCHECKER(occSet,statObs,dynObs)
%
% Description:
%       This function checks if the occupancy set intersects static or
%       dynamic obstacles.
%
% Input Arguments:
%
%       -occSet:    cell-array storing the occupancy set
%       -statObs:   cell-array storing the static obstacles
%       -dynObs:    cell-array storing the dynamic obstacles
%
% Output Arguments:
%
%       -res:       flag specifying if there was a collision or not
%                   (0 if collision, 1 if not)
%
% See Also:
%       maneuverAutomaton
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
% Authors:      Jinyue Guan, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 
   
    res = 1;
    
    % loop over all time intervals of the occupancy set
    for i = 1:length(occSet)
        
        % loop over all static obstacles
        for j = 1:length(statObs)
            if isIntersecting(occSet{i}.set,statObs{j})
                res = 0;
                return 
            end
        end
        
        % loop over all dynamic obstacles
        for k = 1:length(dynObs)
            if isIntersecting(occSet{i}.time,dynObs{k}.time)
                if isIntersecting(occSet{i}.set,dynObs{k}.set)
                    res = 0;
                    return
                end
            end
        end
    end
end