function sol = solveOCP(obj,y0,x0)
%SOLVEOCP - Solves an instance of the optimal control problem.
%
% Syntax:
%       sol = solveOCP(obj,y0,x0)
%
% Description:
%       This function solves the optimal control problem given the initial
%       state x0 of the dynamical system and an initial guess y0 (all
%       optimization variables including the states, control inputs, and
%       slack variables).
%
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -y0:        initial guess
%                   (double array of dimension [obj.idxVars.nVars, 1])
%       -x0:        initial state of the dynamical system
%                   (double array of dimension [obj.nx, 1])
%
% Output Arguments:
%       -sol:       solution to the optimal control problem - structure
%                   with the following fields
%           -.isFeasible:   true, if IPOPT converged to a feasible solution
%                           (boolean)
%           -.x:            optimal state trajectory
%                           (double array of dimension [obj.nx, obj.N+1])
%           -.u:            optimal control input trajectory
%                           (double array of dimension [obj.nu, obj.N])
%           -.slack_bu:     optimal slack variables - softened bounds on
%                           the control inputs
%                           (double array)
%
% See Also:
%       -
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

% indices of optimization variables and constraints
idxVars = obj.idxVars;
idxCon = obj.idxCon;

% bounds on variables
lb = -inf(idxVars.nVars,1);
ub = inf(idxVars.nVars,1);
% ... lower bound on slack variables
lb(idxVars.slack_bu) = - obj.constraintTolerance;

% constraint function bounds
cl = -inf(idxCon.nCon,1);
cu = zeros(idxCon.nCon,1);
% ... lower bound cl = 0 for equality constraints
cl(idxCon.initState) = 0;
cl(idxCon.dynamics) = 0;

% solve OCP --------------------------------------------------------------

res = obj.solver('x0',y0,'p',x0,'lbg',cl,'ubg',cu,'lbx',lb,'ubx',ub);
ySol = full(res.x);
stats = obj.solver.stats();

% extract & post-process solution ----------------------------------------
sol.x = reshape(ySol(idxVars.x),obj.nx,[]);
sol.u = reshape(ySol(idxVars.u),obj.nu,[]);

sol.slack_bu = reshape(ySol(idxVars.slack_bu),[],obj.N);

if max(sol.slack_bu,[],'all') > 1e-6 || ~stats.success  %~ismember(info.status,[0,1])
    sol.isFeasible = false;
    dbstop if warning
    warning('Solution to the optimal control problem might be infeasible!')
else
    sol.isFeasible = true;
end

dbclear if warning
end
