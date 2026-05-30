function obj = computeVariableIndices(obj,U)
%COMPUTEVARIABLEINDICES - Computes the indices optimization variables.
%
% Syntax:
%       obj = computeVariableIndices(obj,U)
% 
% Description:
%       To formulate the optimal control problem (OCP) in the generic form 
%       of a nonlinear programming problem, i.e., 
%           min f(y) s.t. lb <= g(y) <= ub,
%       the variables must be lumped together into a single vector y. To, 
%       e.g., easily extract the states or control inputs from the optimal 
%       solution, this function computes the indices of the entries of y 
%       storing the corresponding OCP variable.
%       These indices are stored in the property idxVars - a structure with
%       the following fields:
%               .x:             (nominal) states
%                               (integer array of dimension [obj.nx,obj.N+1])
%               .xVec:          vectorized idxVars.x
%                               (integer array of dimension [obj.nx*(obj.N+1),1])
%               .u:             (nominal) control inputs
%                               (integer array of dimension [obj.nu,obj.N])
%               .uVec:          vectorized idxVars.u
%                               (integer array of dimension [obj.nu*obj.N,1])
%               .slack_bu:      slack variables associated with violations 
%                               of the bounds on control inputs
%                               (integer array of dimension [U.dim,obj.N])
%               .slack_buVec:   vectorized idxVars.slack_bu
%                               (integer array of dimension [obj.nu*obj.N,1])
%               .nVars:         number of optimization variables
%                               (integer)
%
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -U:         set of admissible control inputs
%                   (class: polytope)
%
% Output Arguments:
%       -obj:       updated instance of class optimalControl    
%
% See Also:
%       optimalControl
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

% init structure storing the indices of the optimization variables
idxVars = struct();

% states
idxVars.xVec = (1:obj.nx*(obj.N+1))';
idxVars.x = reshape(idxVars.xVec,obj.nx,[]);
idxVars.n_x = length(idxVars.xVec);
counter = idxVars.n_x;

% control inputs
idxVars.uVec = (counter + (1:obj.nu*obj.N))';
idxVars.u = reshape(idxVars.uVec,obj.nu,[]);
idxVars.n_u = length(idxVars.uVec);
counter = counter + idxVars.n_u;

% slack variables - (soft contstrained) input bounds
idxVars.slack_buVec = (counter + (1:size(U.A,1)*obj.N))';
idxVars.slack_bu = reshape(idxVars.slack_buVec,[],obj.N);
idxVars.n_slack_bu = length(idxVars.slack_buVec);
counter = counter + idxVars.n_slack_bu;

% number of variables = dimension of the vector
idxVars.nVars = counter;

% set property
obj.idxVars = idxVars;

end
