function [lam_g,lam_x] = computeDualInitialGuess(x0,cost_handle,con_handle,idxVars,idxCon,lbx,ubx)
%COMPUTEDUALINITIALGUESS - computes an initial guess for the dual variables
%       of the polynomial program.
%
% Syntax:
%       [lam_g,lam_x] = computeDualInitialGuess(x0,cost_handle,con_handle,idxVars,idxCon,lbx,ubx)
%
% Description:
%       This function computes an initial guess for the dual variables of
%       the polynomial program in [1, Eq. (19)] by evaluating the KKT
%       conditions at the primal initial guess x0 (thus, implicitly
%       assuming that x0 is a stationary point).
%       The polynomial program is formulated as in the CasADi
%       documentation.
%
% Input Arguments:
%       -x0:            initial guess for the primal variables
%                       (double array of dimension [idxVars.nVars,1])
%       -cost_handle:   handle to the cost function of the polynomial
%                       program
%       -con_handle:     handle to the constraint function of the polynomial
%                       program (does not include variable bounds)
%       -idxVars:       struct storing the variable indices
%       -idxCon:        struct storing the constraint indices
%       -lbx:           lower bound on the primal variables
%       -ubx:           upper bound on the primal variables
%
% Output Arguments:
%       -lam_x:     initial guess for the Lagrange multipliers associated
%                   with the variable bounds
%                   (double array of dimension [idxVars.nVars,1])
%       -lam_g:     initial guess for the Lagrange multipliers associated
%                   with the constraints
%                   (double array of dimension [idxCon.nCon,1])
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/optimizeRCIpoly
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

import casadi.*

x = SX.sym('x',idxVars.nVars,1);

jac_f = Function('jac_f',{x},{jacobian(cost_handle(x),x)});
jac_f0 = full(jac_f(x0))';
jac_g = Function('jac_g',{x},{jacobian(con_handle(x),x)});
jac_g0 = sparse(jac_g(x0))';
jac_g0(:,idxCon.ineqCon) = jac_g0(:,idxCon.ineqCon); 

% check tightness of variable bounds - due to complementarity condition,
% the corresponding Lagrange multiplier is zero for tight constraints
eq_mask = find(withinTol(lbx,ubx,1e-8));
err_lbx = lbx-x0;
err_ubx = x0-ubx;
lb_mask = find(~withinTol(err_lbx,0,1e-8));
ub_mask = find(~withinTol(err_ubx,0,1e-8));

% setup problem struct
nVars_dual = idxCon.nCon+idxVars.nVars;
problem.f = zeros(nVars_dual,1);
problem.H = speye(nVars_dual);
problem.Aeq = [jac_g0, speye(idxVars.nVars); sparse(idxVars.nVars,idxCon.nCon), diag(x0)]; 
problem.beq = [-jac_f0; zeros(idxVars.nVars,1)];
problem.Aineq = [];
problem.bineq = [];
lb = -inf(nVars_dual,1);
lb(idxCon.ineqCon) = 0;
problem.lb = lb;
problem.ub = inf(nVars_dual,1);
% for variable bound multipliers: set multipliers of inactive constraints
% to zero
% -> lower/upper bound inactive (lbx < ubx)
problem.lb(lb_mask+idxCon.nCon) = 0;
problem.ub(ub_mask+idxCon.nCon) = 0;

% solve linear program
[xOpt_dual,~,exitflag,~,~] = CORAlinprog(problem);

if exitflag ~= 1
    % Failed to compute an initial guess for the Lagrange multipliers.
    % - > Multipliers are set to zero.
    lam_g = zeros(idxCon.nCon,1);
    lam_x = zeros(idxVars.nVars,1);
else
    % extract multipliers
    % ... nonlinear constraints
    lam_g = xOpt_dual(1:idxCon.nCon);
    % ... variable bounds 
    lam_x = xOpt_dual(idxCon.nCon+1:end);
    % -> resolve difference (multiplier for upper/lower bound) except for
    % equality constrained multipliers
    lam_x(setdiff(1:idxVars.nVars,eq_mask)) = abs(lam_x(setdiff(1:idxVars.nVars,eq_mask)));
end

end
