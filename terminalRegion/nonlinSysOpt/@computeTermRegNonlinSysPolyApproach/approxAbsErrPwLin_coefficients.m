function coefficients = approxAbsErrPwLin_coefficients(obj,dynamics,candTermReg)
%APPROXABSERRPWLIN_COEFFICIENTS - computes the coefficients of the 
%       piece-wise linear approximation of the abstraction error.
%
% Syntax:
%       coefficients = approxAbsErrPwLin_coefficients(obj,dynamics,candTermReg)
%
% Description:
%       This function computes the semi-edge lengths of an interval
%       enclosing the abstraction error (conservative polynomialization
%       algorithm with tensor order 3). These serve as the coefficients of
%       the piece-wise linear approximation of the abstraction error 
%       proposed in [1, Example 1].
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysPolyApproach
%       -dynamics:          instance of class termRegNonlinSysDynamics                        
%       -candTermReg:       current candidate for the terminal region
%                           (class: polyZonotope)
%
% Output Arguments:
%       -coefficients:  struct storing the coefficients of the piece-
%                       wise linear approximation of the abstraction error
%               .origin:    upper bound on the abstraction error at 0xUxW
%                           (array of dimension [obj.nx,1])
%               .termReg:   upper bound on the abstraction error at 
%                           box(candTermReg)xUxW
%                           (array of dimension [obj.nx,1])
%               .tr         upper bound on the abstraction error at 
%                           box(candTermReg+trust_region)xUxW
%                           (array of dimension [obj.nx,1])
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_con
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant 
%           Sets of Nonlinear Systems Using Polynomial Controller 
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
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
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

coefficients = struct();

% evauate abstraction error at supporting points
% -> expansion point
tmpSet = zonotope(obj.xEq);
coefficients.origin = computeAbsErr(obj,dynamics,tmpSet).sup;
% -> candidate terminal region
tmpSet = zonotope(candTermReg);
coefficients.termReg = computeAbsErr(obj,dynamics,tmpSet).sup;
% -> trust region
tmpSet = zonotope(candTermReg)+obj.tr_set;
coefficients.tr = computeAbsErr(obj,dynamics,tmpSet).sup;

end
