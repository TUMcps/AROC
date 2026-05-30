function [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible,varargin)
%SOLVEOPTPROBLEM - models and solves the convex programming approximation
%       of the terminal region computation problem.
%
% Syntax:
%       [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible)
%       [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible,bloatAbsErr)
%
% Description:
%       This function models and solves the convex programming 
%       approximation in [1, Eq. 17] of the optimization problem described
%       at the beginning of [1, Sec. III], which returns a robust control
%       invariant set of nonlinear dynamical systems with maximum volume.  
%
% Input Arguments:
%       -obj:                   instance of class
%                               computeTermRegNonlinSysLinApproach
%       -dynamics:              instance of class termRegNonlinSysDynamics                        
%       -termReg:               current candidate for the terminal region
%                               (class: termRegNonlinSysLinApproach)
%       -flagInitGuessFeasible: indicates whether termReg admits a feasible
%                               initial guess (or not)- for consistency
%                               with polynomialization algorithm
%                               (boolean)
%       -bloatAbsErr:           (optional) scaling factor for enlarging the 
%                               set of abstraction errors (see the 
%                               verification procedure described in 
%                               [2, Sec. III 2])
%                               (bloatAbsErr >= 1)
%
% Output Arguments:
%       -obj:               updated instance of class
%                           computeTermRegNonlinSysLinApproach
%       -termReg_opt:       updated candidate for the terminal region
%                           (class: termRegNonlinSysLinApproach)
%       -auxData_iter:      additional solution data (e.g. flag indicating
%                           feasibility of the solution)
%                           (class: termRegNonlinSysSolutionAuxData)
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024.
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

if isscalar(varargin)
    bloatAbsErr = varargin{1};
    assert(bloatAbsErr >= 1, ['Scaling factor for enlarging the set of ' ...
        'abstraction errors must be greater or equal than 1.'])
else
    bloatAbsErr = 1;
end

% translate terminal set to expansion point
termRegSet = termReg.set;

% reachable set due to the dynamics evaluated at the equilibrium state and 
% control input
Rfexp = dynamics.Rfexp;
% linearized dynamics
AB_d = dynamics.AB_d;
% reachable set due to the set of disturbances
RlinW = dynamics.RlinW;

% compute trust region
tr = termReg.set + obj.tr_set;
% convert to H-representation
H_tr = polytope(box(tr));

% approximate abstraction error
[quadCoeffW,bilinCoeffW,trCoeffW] = ...
    obj.approxAbsErrQuad_coefficients(dynamics.sys,termReg.set,obj.W);

% interval enclosure of set of admissible inputs and disturbances
IHu = interval(obj.Z_U + (zonotope(-obj.uEq)));
du = max(abs(IHu.sup),abs(IHu.inf));
IHw = interval(obj.W);
dw = max(abs(IHw.inf),abs(IHw.sup));

% convex optimization ----------------------------------------------------

% number of generators of the reachable set
nGen_RparamW = obj.nx*(obj.termRegOrder) + size(RlinW.G,2) ...
    + obj.nx*dynamics.nrTermsInputSolution;

yalmip('clear')
% -------- declare variables --------

if ~obj.flagCostExact
    % scaling factors for opt. set
    var_sCost = sdpvar(size(generators(obj.costSet),2),1,'full');
    var_GammaCost = sdpvar(obj.nx*obj.termRegOrder,size(obj.costSet.G,2),'full');
    var_GammaCost_abs = sdpvar(obj.nx*obj.termRegOrder,size(obj.costSet.G,2),'full');
end
% terminal region
var_s = sdpvar(obj.nx*obj.termRegOrder,1,'full');
var_Gctrl = sdpvar(obj.nu,obj.nx*obj.termRegOrder,'full');
% robust control invariance of the terminal region
var_GammaRCI = sdpvar(obj.nx*obj.termRegOrder,nGen_RparamW,'full');
var_gammaRCI = sdpvar(obj.nx*obj.termRegOrder,1,'full');
var_GammaRCI_abs = sdpvar(obj.nx*obj.termRegOrder,nGen_RparamW,'full');
var_gammaRCI_abs = sdpvar(obj.nx*obj.termRegOrder,1,'full');
% parameterization of the abstraction error
var_auxAbsErrW = sdpvar(obj.nx,1,'full');
var_scaleOut = sdpvar(1,1);
var_dxQuadAux = sdpvar(obj.nx,1);
% slack variables
var_slackRCI = sdpvar(obj.nx*obj.termRegOrder,1,'full');

% ---------- cost function ----------

% penalize constraint violation
objective  = obj.penaltyWeight*sum(var_slackRCI);

constraints = [];

% cost function (volume of terminal region)
% + (if applicable) auxiliary contraints
if ~obj.flagCostExact
    % approx. zonotope volume
    objective = objective - obj.cvx_approxZonoVolume(var_sCost);
    % "cost set" is contained in the terminal region
    [cost_cEq,cost_cIneq] = obj.cvx_zonoContSoftConstraint( ...
        obj.costSet.c,obj.costSet.G*diag(var_sCost), ...
        termRegSet.c,termRegSet.G, ...
        var_GammaCost,zeros(obj.nx*obj.termRegOrder,1), ...
        var_GammaCost_abs,zeros(obj.nx*obj.termRegOrder,1), ...
        var_s,zeros(obj.nx*obj.termRegOrder,1));
    constraints = [constraints, cost_cEq == 0, cost_cIneq <= 0];
else
    % approx. zonotope volume
    objective = objective - obj.cvx_approxZonoVolume(var_s);
end

% ----------- constraints -----------
  
% robust control invariance of terminal set
% ... reachable set due to set of abstraction errors
var_RabsErr = dynamics.computeReachSetAbsErr( ...
    zeros(obj.nx+obj.nu,1),bloatAbsErr*[diag(var_auxAbsErrW);zeros(obj.nu,obj.nx)]);
% ... parameterized reachable set
[c_RparamW,G_RparamW] = obj.cvx_paramReachSet( ...
    var_s,var_Gctrl,var_RabsErr,Rfexp,AB_d,RlinW,termRegSet);
% ... robust control invariance [1, Eq. 17b]
[RCI_cEq,RCI_cIneq] = obj.cvx_zonoContSoftConstraint( ...
    c_RparamW,G_RparamW,termRegSet.c,termRegSet.G, ...
    var_GammaRCI,var_gammaRCI, var_GammaRCI_abs, var_gammaRCI_abs,...
    var_s,var_slackRCI);
constraints = [constraints, ...
    RCI_cEq == 0, RCI_cIneq <= 0];

% abtraction error
% ... box enclosure of augmented terminal region for quadratic terms 
var_dx = obj.cvx_intervalTermReg(var_s,termRegSet);
constraints = [constraints, var_dx <= sqrt(var_dxQuadAux)];
dz_quad = [var_dxQuadAux; du.^2; dw.^2];
% ... trust region exploitation, see also [1, Eq. 28]
constraints = [constraints, var_scaleOut >= ...
    obj.computeTrustRegionExploitation(var_s,termRegSet)];
% ... approximation of abstraction error
constraints = [constraints, ....
    obj.approxAbsErrQuad_eval(dz_quad,var_scaleOut, ...
    quadCoeffW,bilinCoeffW,trCoeffW) - var_auxAbsErrW <= 0];

% state constraint satisfaction [1, Eq. 17c]
constraints = [constraints, obj.cvx_zonoInPolyConstraint(obj.xEq,termRegSet.G,var_s,obj.H_X_con) <= 0];

% input constraint satisfaction [1, Eq. 17d]
constraints = [constraints, obj.cvx_zonoInPolyConstraint( ...
    obj.uEq,var_Gctrl,ones(obj.nx*obj.termRegOrder,1),obj.H_U) <= 0];

% trust-region-like constraint [1, Eq. 17e]
constraints = [constraints, obj.cvx_zonoInPolyConstraint(obj.xEq,termRegSet.G,var_s,H_tr) <= 0];

% auxiliary constraints
constraints = [constraints, 1e-3 <= var_s];
constraints = [constraints, -1e-6 <= var_slackRCI];
constraints = [constraints, 0 <= var_scaleOut];

% solve
settings = sdpsettings('solver','mosek','allownonconvex',false, ...
    'verbose',0);
sol_diagnostics = optimize(constraints,objective,settings);
cvx_time = sol_diagnostics.solvertime;
cvx_setupTime = sol_diagnostics.yalmiptime;

% extract solution
var_s = value(var_s);
if ~obj.flagCostExact
    var_sCost = value(var_sCost);
else
    var_sCost = var_s;
end
var_Gctrl = value(var_Gctrl);
var_slack = value(var_slackRCI);
var_scaleOut = value(var_scaleOut);
dz_quad = [value(var_dx); du; dw].^2;

% assign output arguments ------------------------------------------------

% updata candidate terminal set and invariance-preserving controller
set_opt = zonotope(obj.xEq,generators(termRegSet)*diag(var_s));
ctrl_opt = zonotope(obj.uEq,var_Gctrl);

termReg_opt = termRegNonlinSysLinApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,set_opt,ctrl_opt,dynamics.dt,obj.Z_U,obj.W);

% interval enclosing (the approximation of) the abstraction error
width_absErr = obj.approxAbsErrQuad_eval( ...
    dz_quad,var_scaleOut,quadCoeffW,bilinCoeffW,trCoeffW);
estAbsErr  = interval(-width_absErr,width_absErr);

% exploitation of trust region
tr_exploit = min(obj.computeTrustRegionExploitation(var_s,termRegSet),1);

[obj,auxData_iter] = obj.postprocessIterate(var_sCost,var_slack,tr_exploit, ...
    estAbsErr,cvx_time,cvx_setupTime);

end
