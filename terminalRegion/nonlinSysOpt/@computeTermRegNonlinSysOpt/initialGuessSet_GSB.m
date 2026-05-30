function minRCIset = initialGuessSet_GSB(obj,dynamics)
% INITIALGUESSSET_GSB - computes an outer approximation of the minimal 
%       robust control invariant set of the linearized dynamics.
%
% Syntax:
%       minRCIset = initialGuessSet_GSB(obj,dynamics)
%
% Description:
%       To obtain an initial guess for the optimization-based approaches 
%       for computing terminal regions of nonlinear systems, we compute an 
%       outer approximation of the minimal robust control invariant (RCI) 
%       set [1, Theorem 2]. The cost fuction is taken from [2, Sec. IV].
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -dynamics:          instance of class termRegNonlinSysDynamics
%
% Output Arguments:
%       -minRCIset:     outer approximation of the minimal RCI set
%                       (class: termRegNonlinSysLinApproach)
%           
% See also:
%       -
%
% References:
%       [1] K. Ghasemi et al. "Compositional synthesis via a convex 
%           parameterization of assume-guarantee contracts", 2020, 
%           Proceedings of the 23rd International Conference on Hybrid 
%           Systems: Computation and Control, Article No.: 16, pp. 1-10.
%       [2] K. Ghasemi et al., "Compositional Synthesis of Decentralized 
%           Robust Set-Invariance Controllers for Large-scale Linear 
%           Systems," 2019, IEEE Conference on Decision and Control,
%           pp. 2054-2059.
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

% get linear dynamics
AB_d_nom = dynamics.AB_d.C;
Ad = AB_d_nom(:,1:obj.nx);
Bd = AB_d_nom(:,obj.nx+(1:obj.nu));
RlinW = dynamics.RlinW_dt_nom;

% center the set of disturbances at the origin
RlinW = compact(zonotope(zeros(obj.nx,1),[RlinW.G,RlinW.c]));

% setup optimization problem ---------------------------------------------

yalmip('clear')

% intialize yalmip variables
numGen = obj.termRegOrder*obj.nx;
numGen_W = size(RlinW.G,2);
var_T = sdpvar(obj.nx,numGen,'full');
var_M = sdpvar(obj.nu,numGen,'full');
var_beta = sdpvar(1,1,'full');
var_E = sdpvar(obj.nx,numGen_W,'full');
var_Gamma = sdpvar(numGen_W,numGen_W,'full');
var_slack_u = sdpvar(length(obj.H_U.b),1,'full');
% ... auxiliary variables for resolving absolute values
var_aux_X = sdpvar(size(obj.H_X.A,1),numGen,'full');
var_aux_U = sdpvar(size(obj.H_U.A,1),numGen,'full');

% cost function
cost = norm(var_T,1) + var_beta ...
    + obj.penaltyWeight*sum(var_slack_u,'all');

% constraints
constraints = var_beta >= 1e-6;

% invariance constraint [1, Eq. 15a]
constraints = [constraints, ...
    [Ad*var_T + Bd*var_M, RlinW.G] == [var_E,var_T]];

% auxiliary constraint [1, Eq. 15b]
constraints = [constraints, ...
    var_E == RlinW.G*var_Gamma, ...
    sum(abs(var_Gamma),2) <= var_beta];

% state constraint [1, Eq. 15c]
d_x = obj.H_X.b - obj.H_X.A*obj.xEq;
constraints = [constraints, ...
    obj.H_X.A*var_T <= var_aux_X, -(obj.H_X.A*var_T) <= var_aux_X, ...
    sum(var_aux_X,2) <= (1-var_beta)*d_x];

% input constraint [1, Eq. 15d]
d_u = obj.H_U.b - obj.H_U.A*obj.uEq;
constraints = [constraints, ...
    obj.H_U.A*var_M <= var_aux_U, -(obj.H_U.A*var_M) <= var_aux_U, ...
    sum(var_aux_U,2) <= (1-var_beta)*d_u + var_slack_u];
    % sum(abs(obj.H_U.A*var_M),2) <= (1-var_beta)*d_u];

constraints = [constraints,var_slack_u>=0];

% solve optimization problem and process solution ------------------------

% solve convex program 
diagnostics = optimize(constraints,cost,sdpsettings('verbose',0,'solver','linprog'));

if diagnostics.problem == 0
    var_T = value(var_T);
    var_beta = value(var_beta);
    var_M = value(var_M);
else
    error(['Infeasibility or numerical issues in computation of ', ...
        'initial guess for the terminal region!'])
end

% terminal region and invariance-preserving control law
termReg = zonotope(obj.xEq,var_T/(1-var_beta));
termRegCtrl = zonotope(obj.uEq,var_M/(1-var_beta));

% assign output argument
linDynamicsHandle = @(x,u,w) obj.xEq + Ad*(x-obj.xEq) + Bd*(u-obj.uEq) + w;
minRCIset = termRegNonlinSysLinApproach(linDynamicsHandle,termRegSysType.DT, ...
    termReg,termRegCtrl,dynamics.dt,obj.H_U,RlinW);

end

