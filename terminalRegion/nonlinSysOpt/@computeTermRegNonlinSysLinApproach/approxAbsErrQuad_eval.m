function approxAbsErr = approxAbsErrQuad_eval(var_dzquad,var_exploitTR,quadCoeff,bilinCoeff,trCoeff)
%APPROXABSERRQUAD_EVAL - evaluates the quadratic approximation of the 
%       abstraction error.
%
% Syntax:
%       approxAbsErr = approxAbsErrQuad_eval(var_dzquad,var_exploitTR,quadCoeff,bilinCoeff,trCoeff)
%
% Description:
%       This function evaluates one of the quadratic approximation of the
%       abstraction error proposed in [1, Proposition 1]. 
%
% Input Arguments:
%       -var_dzquad:    squared semi-edge lengths of interval enclosing the 
%                       Cartesian product of the terminal region, the set 
%                       of admissible control inputs and the set of 
%                       disturbances 
%                       (array of dimension [nx+nu+nw,1])
%       -var_exploitTR: scalar measuring the exploitation of the trust 
%                       region, see [1, Eq. 28a]
%       -quadCoeff:     coefficients of the quadratic terms [1, Eq. 27a]
%                       (array of dimension [nx,1])
%       -bilinCoeff:    coefficients of the bilinear terms in [1, Eq. 27a]
%                       (array of dimension [nx,1])
%       -trCoeff:       coefficients of the linear term, see [1, Eq. 27b]
%                       (array of dimension [nx,1])
%
% Output Arguments:
%       -approxAbsErr:  (lower bound of) abstraction error approximation
%                       (non-negative array of dimension [nx,1])
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/approxAbsErrQuad_coefficients
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

approxAbsErr = quadCoeff*var_dzquad + bilinCoeff + trCoeff*var_exploitTR;

end
