function obj = computeConstraintIndices(obj,X,U)
%COMPUTECONSTRAINTINDICES - Computes the indices of the individual constraint functions.
%
% Syntax:
%       obj = computeConstraintIndices(obj,X,U)
%
% Description:
%       To formulate the optimal control problem (OCP) in the generic form 
%       of a nonlinear programming problem, i.e., 
%           min f(y) s.t. lb <= g(y) <= ub,
%       the equality and inequality constraints have to be lumped into a 
%       single vector-valued fuction. To, e.g., easily access the value of
%       individual constraint functions, this function computes the indices
%       of the entries storing the value of the respective constraint
%       function.
%       These indices are stored in the property idxCon - a structure with
%       the following fields:
%           .initState:     initial state constraint
%                           (integer array of dimension [obj.nx,1])
%           .dynamics:      dynamic feasibility
%                           (integer array of dimension [obj.nx,obj.N])
%           .bu:            admissibility of the control inputs
%                           (integer array of dimension [U.dim,obj.N])
%           .bx:            admissibility of the states
%                           (integer array of dimension [X.dim,obj.N+1])
%           .nIneqCon:      number of inequality constraints
%                           (integer)
%           .nEqCon:        number of equality constraints
%                           (integer)
%           .nCon:          total number of constraints
%                           (integer)
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

% init structure storing the indices of the constraints
idxCon = struct();
% init counter
countCon = 0;

% equality constraints ---------------------------------------------------
% initial state
idxCon.initState = (countCon + (1:obj.nx))';
countCon = countCon + obj.nx;

% dynamic feasibility: x_{k+1} = f(x_k,u_k)
idxCon.dynamics = reshape(countCon + (1:obj.nx*obj.N),obj.nx,obj.N);
countCon = countCon + obj.nx*obj.N;

idxCon.nEqCon = countCon;

% inequality constraints -------------------------------------------------

nCon_x = size(X.A,1);
nCon_u = size(U.A,1);

% 0 <= k <= N-1
nConStep = nCon_x + nCon_u;
tmp = countCon + reshape(1:obj.N*nConStep,[],obj.N);
idxCon.bu = tmp(1:nCon_u,:);
idxCon.bx = tmp(nCon_u+(1:nCon_x),:);
countCon = countCon + obj.N*nConStep;

% k = N
tmp = (countCon + (1:nCon_x))';
idxCon.bx = [idxCon.bx,tmp];
countCon = countCon + length(tmp);

% number of constraints
idxCon.nCon = countCon;
idxCon.nIneqCon = countCon - idxCon.nEqCon;

% set property
obj.idxCon = idxCon;

end
