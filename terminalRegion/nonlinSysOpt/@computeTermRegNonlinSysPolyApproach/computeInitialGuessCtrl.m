function p0 = computeInitialGuessCtrl(obj,dynamics,candTermReg)
%COMPUTEINITIALGUESSCTRL - computation of an initial guess for the controller
%       parameters.
%
% Syntax:
%        p0 = computeInitialGuessCtrl(obj,dynamics,candTermReg)
%
% Description:
%       This function computes an initial guess for the controller 
%       parameters by solving a based on the controller synthesis approach 
%       from [2]. To enable the algorithm to recover feasibility faster, 
%       the goal is to steer all states of the candidate terminal region 
%       back into this set with minimum control effort (encoded as the 
%       exploitation of set of admissible inputs). To formulate a convex 
%       program, we only cosider the linear abstraction of the dynamics.
%
% Input Arguments:
%       -obj:               instance of class 
%                           computeTermRegNonlinSysPolyApproach
%       -dynamics:          instance of class termRegNonlinSysDynamics   
%       -candTermReg:       initial guess for the terminal region
%                           (class: polyZonotope)
%
% Output Arguments:
%       - p0:   initial guess for the controller parameters
%               (double array of dimension [length(obj.id.p),1])
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/computeInitialGuess
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169. 
%       [2] B. Schürmann and M. Althoff, "Guaranteeing Constraints of 
%           Disturbed Nonlinear Systems Using Set-Based Optimal Control in 
%           Generator Space,", IFAC-PapersOnLine, Volume 50, Issue 1, 2017,
%           pp. 11515-11522.
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

% extract linear (in the terminal region factors) exponential vectors from
% controller template
[tmp_idv, idx_idv,~] = intersect(obj.pZu_vp.id,obj.id.termReg,'stable');
idx_lin = find(sum(obj.pZu_vp.E(idx_idv,:),1) <= 1);

% remove unctrolled generators (with linear exponential vectors)
[tmp_idp,idx_idp,~] = intersect(obj.pZu_vp.id,obj.id.p,'stable');
idx_idp_lin = find(sum(obj.pZu_vp.E(idx_idp,idx_lin),2));

% set linearized controller template
pZu_vp_lin = polyZonotope(obj.pZu_vp.c,obj.pZu_vp.G(:,idx_lin),[], ...
    obj.pZu_vp.E([idx_idp_lin;idx_idv],idx_lin), ...
    [tmp_idp(idx_idp_lin);tmp_idv]);

% approximate reachable set
% ... initial set
R0 = stack(candTermReg,pZu_vp_lin);
% ... approximate time-point reachable set
R_lin = project(dynamics.AB_d.C*R0 + dynamics.RlinW_dt_nom + dynamics.Rfexp.C, ...
    1:obj.nx);
% R_lin = approxReachableSet(candTermReg,pZu_vp_lin,opts.xEq,opts.uEq,opts);

% setup and solve optimization problem 
p0 = computeInitialP(obj,R_lin,pZu_vp_lin,candTermReg);

end


%% auxiliary functions

function p0 = computeInitialP(solver,R_lin,pZu_vp_lin,candTermReg)
% In this auxiliary function, we setup and solve the convex program, which
% returns the initial guess for the controller parameters.
%
% ------------------------------------------------------------------------

% partial zonotope over-approximation of dependent part of R_lin
Rdep = polyZonotope(R_lin.c,R_lin.G,[],R_lin.E,R_lin.id);
[~,Rdep_gi,~,~,~] = partZonotope(Rdep, ...
    intersect(Rdep.id,solver.id.termReg,'stable'));
[idp_lin,~,idx_idp_lin] = intersect(R_lin.id,solver.id.p,'stable');

nGenR = (length(Rdep_gi)-1) + size(R_lin.GI,2);
nGenTermReg = size(candTermReg.G,2);

% partial zonotope over-approximation of controller template
[~,pZu_gi_cell,~,~,~] = partZonotope( ...
    pZu_vp_lin,intersect(pZu_vp_lin.id,candTermReg.id));

yalmip('clear')

% initialize yalmip variables
var_p = sdpvar(length(idp_lin),1,'full');
var_Gamma = sdpvar(nGenTermReg,nGenR,'full');
var_gamma = sdpvar(nGenTermReg,1,'full');
var_slack = sdpvar(nGenTermReg,1,'full');
var_exploitX = sdpvar(1,1,'full');
var_resolveAbsValU = sdpvar(size(solver.H_U.A,1),length(pZu_gi_cell)-1,'full');

% cost function
cost = -var_exploitX + solver.penaltyWeight*sum(var_slack);

% constraints
constraints = [];

% invariance constraint - dependent generators
for ii = 2:length(Rdep_gi)
    [~,tmp_idx,~] = intersect(idp_lin,Rdep_gi{ii}.id,'stable');
    constraints = [constraints, ...
        Rdep_gi{ii}.c + Rdep_gi{ii}.G*(var_p(tmp_idx)'*Rdep_gi{ii}.E)' ...
        == candTermReg.G*var_Gamma(:,ii-1)];
end
% invariance constraint - independent generators
constraints = [constraints, ...
    R_lin.GI == candTermReg.G*var_Gamma(:,length(Rdep_gi):end)];
% invariance constraint - center
[~,tmp_idx,~] = intersect(idp_lin,Rdep_gi{1}.id,'stable');
constraints = [constraints, ...
        center(candTermReg) - (Rdep_gi{1}.c + Rdep_gi{1}.G*(var_p(tmp_idx)'*Rdep_gi{1}.E)') ...
        == candTermReg.G*var_gamma];
% invariance constraint - norm condition
constraints = [constraints, ...
    sum(abs([var_Gamma,var_gamma]),2) <= 1+var_slack-var_exploitX, ...
    var_slack >= 0, var_exploitX >= 0];

% constraints - inputs for center of cand. terminal region
% assumption: uRef constant over entire time step
ac = generators(solver.Z_U)\(solver.uEq(:,1) - center(solver.Z_U));
[~,idx_idp_c,~] = intersect(idp_lin, solver.id.pc,'stable');
constraints = [constraints, var_p(idx_idp_c) == ac(:)];

% setup input constraints
[~,tmp_idx,~] = intersect(idp_lin,pZu_gi_cell{1}.id,'stable');
% setup input constraint - center
tmp_constraint = solver.H_U.A*(pZu_gi_cell{1}.c + pZu_gi_cell{1}.G*(var_p(tmp_idx)'*pZu_gi_cell{1}.E)');
% setup input constraint - dependent generators (resolve absolute values)
H_UA = solver.H_U.A;
for ii = 2:length(pZu_gi_cell)
    [~,tmp_idx,~] = intersect(idp_lin,pZu_gi_cell{1}.id,'stable');
    tmp = pZu_gi_cell{ii}.c + pZu_gi_cell{ii}.G*(var_p(tmp_idx)'*pZu_gi_cell{ii}.E)';
    constraints = [constraints, ...
        H_UA*tmp <= var_resolveAbsValU(:,ii-1),
        -H_UA*tmp <= var_resolveAbsValU(:,ii-1)
        ];
end
tmp_constraint = tmp_constraint + sum(var_resolveAbsValU,2);
% setup input constrait - finalize constraint
constraints = [constraints, tmp_constraint <= solver.H_U.b];

% solve convex program using YALMIP
diagnostics = optimize(constraints,cost,sdpsettings('verbose',0, ...
    'allownonconvex',0));

if diagnostics.problem == 0
    candP_lin = value(var_p);
else
    error(['Infeasibility or numerical issues in computation of ', ...
        'initial guess for controller parameters!'])
end

% assign output values 
p0 = zeros(solver.ctrl.np,1);
p0(idx_idp_lin) = candP_lin;

end
