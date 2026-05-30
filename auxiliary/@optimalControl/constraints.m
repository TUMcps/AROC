function con = constraints(obj,y,x0,X,U)
%CONSTRAINTS - Constraint function of the optimal control problem.
%
% Syntax:
%       con = constraints(obj,y,x0,X,U)
%
% Description:
%       Constraints of the optimal control problem in the form  
%       lb <= constraints(obj,y,x0,X,U) = g(y) <= ub.
%
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -y:         vector of optimization variables
%                   (double array of dimension [obj.idxVars.nVars,1])
%       -x0:        initial state
%       -X:         set of admissible states
%                   (class: polytope)
%       -U:         set of admissible control inputs
%                   (class:polytope)   
%
% Output Arguments:
%       -cost:      value of constraint function evaluated at y
%                   (double / casadi SX symbolic)
%   
% See Also:
%       @optimalControl/computeConstraintIndices
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

% extract variables
idxVars = obj.idxVars;
x = reshape(y(idxVars.x),obj.nx,obj.N+1);
u = reshape(y(idxVars.u),obj.nu,obj.N);
slack_bu = reshape(y(idxVars.slack_bu),idxVars.n_slack_bu/obj.N,obj.N);

idxCon = obj.idxCon;

% initialize vector collecting the constraints
if ~isnumeric(y)
    % symbolic variable for automatic differentiation of the constraint 
    % function using CasADi,
    con = casadi.SX.sym('con',idxCon.nCon,1);
else
    con = zeros(idxCon.nCon,1);
end

% initial state constraint
con(idxCon.initState) = x(:,1) - x0;

% state constraints: \forall k: x_k \in X 
con(idxCon.bx) = X.A*x - X.b;

% (softened) input constraints: \forall k u_k \in U
con(idxCon.bu) = U.A*u - U.b + slack_bu;

% stage constraints (0 <= kk <= N-1)
for kk = 1:obj.N
    % ... dynamic feasibility
    tmp_ceq = obj.dynamics_dt(x(:,kk),u(:,kk)) - x(:,kk+1);
    con(idxCon.dynamics(:,kk)) = full(tmp_ceq);
end

end
