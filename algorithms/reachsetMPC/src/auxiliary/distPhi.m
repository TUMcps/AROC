function d = distPhi(A,b,zono)
% DISTPHI - distance between a zonotope and the terminal region
%
% Syntax:
%       d = DISTPHI(A,b,zono)
%
% Description:
%       This function computes the distance between a zonotope and the
%       terminal region, which is specified as a polytope (see Def. 5 in
%       [1]).
%
% Input Arguments:  
%
%       -A:     inequality constraint matrix the defines the terminal 
%               region polytope (A*x < b) 
%       -b:     inequality constraint vector the defines the terminal 
%               region polytope (A*x < b) 
%       -zono:  zonotope (class: zonotope)
%
% Output Arguments:
%
%       -d:     distance of the polytope to the terminal region
%
% See Also:
%       reachsetMPC
%
% References:
%       * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
%              for Disturbed Nonlinear Systems
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


    % extract zonotope parameters
    c = center(zono);
    G = generators(zono);

    % compute distance
    temp1 = A*c;
    temp2 = sum(abs(A*G),2);
    
    d = max(0,max(diag(1./b)*(temp1-b+temp2)));
end