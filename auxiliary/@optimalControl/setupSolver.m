function obj = setupSolver(obj,X,U)
%SETUPSOLVER - Creates a solver for the optimal control problem.
%
% Syntax:
%       obj = setupSolver(obj,X,U)
%
% Description:
%       This function creates a solver for the optimal control problem
%       using CasADi's [2] 'ipopt' plugin, which provides an interface to
%       IPOPT [1].
% 
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -X:         set of admissible states
%                   (class: polytope)
%       -U:         set of admissible control inputs
%                   (class:polytope) 
%
% Output Arguments:
%       -obj:       updated instance of class optimalControl
%   
% See Also:
%       optimalControl
%
% References:
%       [1] A. Wächter and L. T. Biegler, 'On the Implementation of a 
%           Primal-Dual Interior Point Filter Line Search Algorithm for 
%           Large-Scale Nonlinear Programming', 
%           Mathematical Programming 106(1), pp. 25-57, 2006
%       [2] J. A. E. Andersson et al., 'CasADi – A software framework 
%           for nonlinear optimization and optimal control', 
%           Mathematical Programming Computation, vol. 11, no. 1, 
%           pp. 1–36, 2019.
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

% define casadi variables
y = SX.sym('y',obj.idxVars.nVars,1);
% define parameter -> initial state
x0 = SX.sym('x0',obj.nx,1);

%  IPOPT algorithm settings
ipoptOptions.ipopt.constr_viol_tol = obj.constraintTolerance;
ipoptOptions.ipopt.acceptable_constr_viol_tol = obj.constraintTolerance;
ipoptOptions.ipopt.bound_relax_factor = 0;
ipoptOptions.ipopt.compl_inf_tol = 1e-6;
ipoptOptions.ipopt.print_level = 0; % no console output of iterations
ipoptOptions.print_time = 0; % no console output of computation times

% construct NLP
nlp = struct('x',y, ...
    'p',x0, ...
    'f',obj.costFunction(y), ...
    'g',obj.constraints(y,x0,X,U));
obj.solver = nlpsol('S','ipopt',nlp,ipoptOptions);

end
