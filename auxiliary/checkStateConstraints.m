function checkStateConstraints(R,X)
% CHECKSTATECONSTRAINTS - check if the reachable set satisfies the state
%                         constraints
%
% Syntax:
%       CHECKSTATECONSTRAINTS(R,X)
%
% Description:
%       Checks if the reachable set of the controlled system satisfies the
%       state constraints.
%
% Input Arguments:
%
%       -R:     reachable set of the controlled system (class: reachSet)
%       -X:     set of state constraints (class: mptPolytope)
%
% See Also:
%       generatorSpaceControl, convexInterpolationControl
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

    % loop over all reachable sets
    for i = 1:size(R,1)
       for j = 1:length(R(i).timeInterval.set)
          if ~in(X,R(i).timeInterval.set{j})
             error('The state constraints are violated!'); 
          end
       end
    end
    
end