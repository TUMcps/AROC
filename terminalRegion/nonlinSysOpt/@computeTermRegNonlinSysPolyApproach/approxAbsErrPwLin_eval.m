function approxAbsErr = approxAbsErrPwLin_eval(obj,candTermReg_sc,s,delta_c,paramAbsErr_coefficients)
%APPROXABSERRPWLIN_EVAL - evaluation function for the piece-wise linear
%       approximation of the abstraction error.
%
% Syntax:
%       approxAbsErr = approxAbsErrPwLin_eval(obj,candTermReg_sc,s,delta_c,paramAbsErr_coefficients)
%
% Description:
%       This function returns the value of the piece-wise linear 
%       approximation of the abstraction error proposed in [1, Example 1].
%
% Input Arguments:
%       -obj:                       instance of class
%                                   computeTermRegNonlinSysPolyApproach
%       -candTermReg_sc:            parameterized candidate terminal region
%                                   (center and generator scaling factors)
%                                   (polyZonotope/struct storing the 
%                                   parameters of a polyZonotope)
%       -s:                         generator scaling factors for the
%                                   candidate terminal region
%                                   (double vector)
%       -delta_c:                   center translation for the canddiate 
%                                   terminal region
%                                   (array of dimension [obj.nx,1])
%       -paramAbsErr_coefficients:  struct storing the coefficients of 
%                                   the piece-wise linear approximation of 
%                                   the abstraction error
%               .origin:    upper bound on the abstraction error at 0xUxW
%                           (array of dimension [obj.nx,1])
%               .termReg:   upper bound on the abstraction error at 
%                           box(candTermReg)xUxW
%                           (array of dimension [obj.nx,1])
%               .tr         upper bound on the abstraction error at 
%                           box(candTermReg+trust_region)xUxW
%                           (array of dimension [obj.nx,1])
%
% Output Arguments:
%       -approxAbsErr:  (lower bound of) abstraction error approximation
%                       (non-negative array of dimension [obj.nx,1])
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_coefficients
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
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% compute optimized terminal region
pZ_sOpt = polyZonotope(zeros(length(obj.id.termReg),1),diag(s),[], ...
    eye(length(obj.id.termReg)),obj.id.termReg);
candTermReg_c = subs(candTermReg_sc,pZ_sOpt,obj.id.termReg);
termRegOpt = resolve(candTermReg_c,delta_c,obj.id.delta_c);

% candidate terminal region (=initial guess)
candTermReg = resolve(candTermReg_sc,zeros(obj.nx,1),obj.id.delta_c);

% compute uniform scaling factor of candTermReg to contain the optimized
% terminal region
% ... interval enclosures centered at the equilibrium
delta_IHcand = interval(candTermReg) - obj.xEq;
dx0 = max([abs(delta_IHcand.inf),abs(delta_IHcand.sup)],[],2);
dx0 = dx0(obj.argAbsErr);
delta_IHopt = interval(termRegOpt) - obj.xEq;
dx_opt = max([abs(delta_IHopt.inf),abs(delta_IHopt.sup)],[],2);
dx_opt = dx_opt(obj.argAbsErr);
auxVar_inner = max(dx_opt./dx0);

% compute exploitation of trust region
tr = interval(-dx0,dx0) + project(obj.tr_set,obj.argAbsErr);
dx_tr = interval(tr).sup;
auxVar_outer = max((dx_opt-dx0)./(dx_tr-dx0));

% evaluate abstraction error approximation
approxAbsErr = max([ ...
    paramAbsErr_coefficients.origin*(1-auxVar_inner) + paramAbsErr_coefficients.termReg*auxVar_inner, ...
    paramAbsErr_coefficients.termReg*(1-auxVar_outer) + paramAbsErr_coefficients.tr*auxVar_outer],[],2);

end