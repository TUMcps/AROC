function cIneq = cvx_zonoInPolyConstraint(cIn,GIn,s,polyOut)
%CVX_ZONOINPOLYCONSTRAINT - implements the constraint function encoding
%       the necessary and sufficient conditions for zonotope-in-polytope 
%       containment.
%
% Syntax:
%       [cEq,cIneq] = cvx_zonoInPolyConstraint(cIn,GIn,s,polyOut)
% 
% Description:
%       This function implements the necessary and sufficient conditions 
%       for containment of a set zonotope(cIn,GIn*diag(s)) in a polytope in
%       [1, Eq. 9]:
%               polytope.A*cIn + abs(polytope.A*GIn)*s <= polytope.b.
%       The output argument is in the form 
%           cIneq <= 0.
%
% Input Arguments:
%       -cIn:       center of the inbody
%                   (double array of dimension [nx,1])
%       -GIn:       generator matrix of the inbody
%                   (double array)
%       -s:         generator scaling factors for the inbody
%                   (double array of dimension [size(GIn,2),1])
%       -polyOut:   circumbody
%                   (class: polytope)
%
% Output Arguments:
%       -cIneq:     constraint functions of the inequality constraints
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

cIneq = polyOut.A*cIn + abs(polyOut.A*GIn)*s ...
        - polyOut.b;

end
