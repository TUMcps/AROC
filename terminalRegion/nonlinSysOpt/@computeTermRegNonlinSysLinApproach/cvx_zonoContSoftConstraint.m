function [cEq,cIneq] = cvx_zonoContSoftConstraint(cIn,GIn,cOut,GOut,Gamma,gamma,Gamma_abs,gamma_abs,ub,slack)
%CVX_ZONOCONTSOFTCONSTRAINT - implements the constraint functions encoding
%       the (softened) sufficient conditions for zonotope containment.
%
% Syntax:
%       [cEq,cIneq] = cvx_zonoContSoftConstraint(cIn,GIn,cOut,GOut,Gamma,gamma,Gamma_abs,gamma_abs,ub,slack)
% 
% Description:
%       This function implements the sufficient conditions for zonotope
%       containment from [2, Corollary 4] in the softened version presented
%       in [1, Remark 4], i.e.
%               GIn = Gout*Gamma
%               cOut - cIn = GOut*gamma
%               norm([gamma, Gamma],'inf') <= ub + slack
%               slack >= 0
%       The input argument ub can either be a vector full of ones (for
%       fixed circumbodies zonotope(cOut,Gout), see [2, Corollary 4]) or a
%       vector of generator scaling factors for variable circumbodies of
%       the form zonotope(cOut,Gout*diag(ub)), see [1, Remark 4]).
%       The output arguments are in the form 
%           Ceq = 0, cIneq >= 0.
%
% Input Arguments:
%       -cIn:       center of the inbody
%                   (double array of dimension [nx,1])
%       -GIn:       generator matrix of the inbody
%                   (double array)
%       -cOut:      center of the inbody
%                   (double array of dimension [nx,1])
%       -GOut:      generator matrix of the inbody
%                   (double array)
%       -Gamma:     auxiliary variable
%                   (double array of dimension [size(GOut,2),size(GIn,2)])
%       -gamma:     auxiliary variable
%                   (double array of dimension [size(GOut,2),1])
%       -Gamma_abs: absolute value of Gamma
%       -gamma_abs: absolute value of gamma
%       -ub:        upper bound in the norm condition
%                   (double array of dimension [size(GOut,2),1])
%       -slack:     slack variable to soften upper bound in the norm 
%                   condition
%                   (double array of dimension [size(GOut,2),1])
%
% Output Arguments:
%       -cEq:       constraint functions of the equality constraints
%       -cIneq:     constraint functions of the inequality constraints
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
%       [2] S. Sadraddini and R. Tedrake, "Linear Encodings for Polytope 
%           Containment Problems," IEEE Conference on Decision and Control, 
%           2019, pp. 4367-4372.
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

cEq = [cOut - cIn - GOut*gamma, GIn - GOut*Gamma];

cIneq = [Gamma - Gamma_abs, -Gamma - Gamma_abs, ...
    gamma - gamma_abs, -gamma - gamma_abs, ...
    sum([Gamma_abs, gamma_abs],2) - ub - slack];

end
