function [sol,fVal] = optimizeRCIpoly(obj,termReg,approx_R,mapsParamReduce,paramAbsErr_coefficients,p0)
%OPTIMIZERCIPOLY - solves the polynomial programming approximation of the 
%       terminal region computation problem.
%
% Syntax:
%       [sol,fVal] = optimizeRCIpoly(obj,termReg,approx_R,mapsParamReduce,paramAbsErr_coefficients,p0)
%
% Description:
%       This function models and solves the polynomial programming 
%       approximation in [1, Eq. (19)] of the optimization problem in 
%       [1, Eq. (6)], which returns a robust control invariant set of 
%       a given nonlinear dynamical system with maximum volume.  
%
% Input Arguments:
%       -obj:                       instance of class
%                                   computeTermRegNonlinSysPolyApproach
%       -termReg:                   initial guess for the terminal region 
%                                   (class: polyZonotope)
%       -approx_R:                  parameterized approximation of the
%                                   time-pointreachable set (see [1,
%                                   Eq. (17)]
%                                   (class: polyZonotope)
%       -mapsParamReduce:           struct storing the parameterization
%                                   of the lengths of the reducing 
%                                   generators in the optimization 
%                                   variables
%       -paramAbsErr_coefficients:  struct storing the coefficients of
%                                   the parameterized approximation of the
%                                   abstraction error
%       -p0:                        initial guess for the controller
%                                   parameters
%                                   (double vector)
%
% Output Arguments:
%       -sol:           struct storing the solution of the polynomial 
%                       program
%                       .p: controller parameters
%                       .delta_c: translation of the terminal region center
%                       .s: terminal region generator scaling factors
%                       .sCost: cost set generator scaling factors
%                       .sRed: reducing generator scaling factors
%                       .sAbsErr: abstraction error interpolation factors
%                       .exploitTR: scalar indicating the trust region
%                           exploitation
%                       .slack: slack variables of softened constraints
%       -fVal:          value of the nominal cost function 
%                       (size of sol.costSet)
%
% See Also:
%       -
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
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

import casadi.*

time_setup = tic;

% pre-processing ---------------------------------------------------------

% extract ids
id_all = obj.id;

% ids of optimization variables
id_opt = [id_all.var_scalar; id_all.var_scaling];

% set for tightening the invariance constraint
tightSet = zonotope(zeros(obj.nx,1),obj.constraintTolerance*eye(obj.nx));
% ... add to reachable set
approx_R = approx_R + tightSet;

% partial zonotope over-approximation of the parameterized reachable set 
R_po = obj.paramPartZonotope(approx_R,id_all.var_scaling,id_all.var_scalar);

% partial zonotope over-approximation of the set representing the
% controller
pZu_vp_po = obj.paramPartZonotope(obj.pZu_vp,id_all.var_scaling,id_all.var_scalar);

% partial zonotope over-approximation of candidate terminal region
termReg_po = preprocessCandTermReg(termReg, ...
    id_all.var_scaling,id_all.var_scalar);

% compute indices of optimization variables
nGenR = length(R_po.G);
nGenCtrl = length(pZu_vp_po.G);
nGenTermReg = size(termReg_po.G,2);
nGenU = size(generators(obj.Z_U),2);
nGenReduce = length(find(~mapsParamReduce.isConstGen));
nGenErr = size(R_po.GI,2);
idxVars = obj.computeVariableIndices(id_all,mapsParamReduce.id,nGenR,nGenTermReg, ...
    nGenCtrl,nGenU,nGenReduce,nGenErr);

% compute function handles and gradients of zonotope enclosures
% -> approximation of reachable set (i.e. R_po)
[R_po,Rpo_funHan] = computeGeneratorHandles(R_po,id_opt,id_all.var_scaling);
% -> controller
[pZctrl_po,pZctrl_funHan] = computeGeneratorHandles(pZu_vp_po,id_opt,id_all.var_scaling);

% compute fuction handles and gradients of maps for parameterized reduction
[mapsRed,mapsParamReduce_funHan] = ...
    computeParamReductionHandles(mapsParamReduce,id_opt,id_all.var_scaling);

% cost function ----------------------------------------------------------
nomCostFun = setupNominalCost(obj,id_all);

% setup constraints ------------------------------------------------------

% lower bounds
lb = -inf(idxVars.nVars,1);
lb(idxVars.s) = 1e-3;
lb(idxVars.slack) = -obj.constraintTolerance; %-1e-12;
lb(idxVars.sAbsErr) = 0;
lb(idxVars.sCost) = 1e-3;
lb(idxVars.auxSRed) = 0;

% upper bounds
ub = inf(idxVars.nVars,1);

% fix scaling factors of reducing generators that are independent of the
% optimization variables
lb(idxVars.sRedConst) = 1;
ub(idxVars.sRedConst) = 1;
idx = find(withinTol(paramAbsErr_coefficients.tr,0,obj.constraintTolerance));
ub(idxVars.sAbsErr(idx)) = 1;

% linear constraints
[Aineq,bineq,Aeq,beq] = linearConstraints(obj,termReg_po, ...
    idxVars,nGenR,nGenTermReg,nGenCtrl,nGenU,R_po.GI);

% parameterized abstraction error
[absErr_con, absErr_conAux] = obj.approxAbsErrPwLin_con(...
    termReg_po,paramAbsErr_coefficients,idxVars);

% setup nonlinear constraints
[ceqStruct,cineqStruct] ...
    = setupNonlinearConstraints(R_po,pZctrl_po,termReg_po,obj.Z_U, ...
    mapsRed,nGenR,nGenCtrl,nGenTermReg,nGenU,nGenReduce, ...
    id_opt,idxVars,obj.nx,obj.nu);

% merge linear and nonlinear constraints
[cl,cu,idxCon] = mergeConstraints( ...
    ceqStruct,cineqStruct,Aeq,beq,Aineq,bineq,absErr_con,absErr_conAux);

% constraint and cost function for IPOPT interface -----------------------

% evaluate cost
funcs.objective = @(x) evalCost(x,nomCostFun,idxVars, ...
    obj.penaltyWeight);

% evaluate constraints
funcs.constraints = @(x) evalConstraints(x, ...
    ceqStruct,cineqStruct,Aeq,Aineq,absErr_con,absErr_conAux,idxVars);

% initial guess computation ----------------------------------------------

% initial guess
y0 = computePrimalInitialGuess(obj,termReg_po,Rpo_funHan,R_po.GI, ...
    pZctrl_funHan,mapsParamReduce_funHan,nGenReduce,p0,idxVars);

% push slack into feasible region (interior point method)
y0(idxVars.slack) = max(y0(idxVars.slack),obj.constraintTolerance);

% check recursive feasibility
con0 = funcs.constraints(y0);
if ~(max(cl - con0) <= 1e-6 ...
        && max(con0 - cu) <= 1e-6 ...
        && y0(idxVars.slack) <= 1e-6)
    % warning('initial guess infeasible!')
    flagInitGuessFeasible = false;
else
    flagInitGuessFeasible = true;
end

% setup solver -----------------------------------------------------------

% IPOPT settings
ipoptOptions.print_time = 0;
ipoptOptions.ipopt.print_level = 0;
ipoptOptions.ipopt.hessian_approximation = 'exact';
ipoptOptions.calc_lam_x = 1;
% ... constraint tolerances
ipoptOptions.ipopt.constr_viol_tol = obj.constraintTolerance;
ipoptOptions.ipopt.acceptable_constr_viol_tol = obj.constraintTolerance;
ipoptOptions.ipopt.bound_relax_factor = 1e-6;
ipoptOptions.ipopt.compl_inf_tol = 1e-6; 

% ... barrier parameter update strategy
ipoptOptions.ipopt.mu_strategy = 'adaptive';
ipoptOptions.ipopt.mu_oracle = 'quality-function';

% ... initial step
ipoptOptions.ipopt.bound_push = 1e-6;
ipoptOptions.ipopt.bound_frac = 1e-6;

% ... additional options to ensure recursive feasibility
if flagInitGuessFeasible
    ipoptOptions.ipopt.warm_start_init_point = 'yes';
    ipoptOptions.ipopt.warm_start_entire_iterate = 'yes';
    ipoptOptions.ipopt.mu_init = 0.001;  % initial value of barrier parameter

    % ... initial step: dual warm start
    ipoptOptions.ipopt.warm_start_bound_push = 1e-6;
    ipoptOptions.ipopt.warm_start_mult_bound_push = 1e-6;
    ipoptOptions.ipopt.warm_start_slack_bound_push = 1e-6;

    % ... change barrier parameter update strategy to avoid being pushed to
    % far away from initial guess
    ipoptOptions.ipopt.mu_strategy = 'monotone';
    ipoptOptions.ipopt.accept_every_trial_step = 'yes';

    ipoptOptions.ipopt.start_with_resto = 'no';
end

% evaluate cost and constraints symbolically 
cost_handle = @(x)funcs.objective(x);
con_handle = @(x)funcs.constraints(x);
x = SX.sym('x',idxVars.nVars,1);
nlp = struct(...
    'x', x, ...
    'f', cost_handle(x), ...
    'g', con_handle(x));
S = nlpsol('S', 'ipopt', nlp, ipoptOptions);

sol.setupTime = toc(time_setup);

% solve NLP using IPOPT --------------------------------------------------

if flagInitGuessFeasible
    % compute initial guess for dual variables
    [lam_g0,lam_x0] = obj.computeDualInitialGuess(y0,cost_handle,con_handle,idxVars,idxCon,lb,ub);
    % solve optimization problem
    tOpt = tic;
    res = S('x0',y0,'lbg',cl,'ubg',cu,'lbx',lb,'ubx',ub,'lam_x0',lam_x0,'lam_g0',lam_g0);
    sol.solverTime = toc(tOpt);
else
    tOpt = tic;
    res = S('x0',y0,'lbg',cl,'ubg',cu,'lbx',lb,'ubx',ub);
    sol.solverTime = toc(tOpt);
end
ySol = full(res.x);
stats = S.stats();


% post-processing --------------------------------------------------------

if ~stats.success
    dbstop if warning
    warning('Solution to optimization problem might be infeasible - please check solution!')
    dbclear if warning
end

% extract scaling factors, center shift, and controller parameters
sol.p = ySol(idxVars.p);
sol.delta_c = ySol(idxVars.delta_c);
sol.s = ySol(idxVars.s);
sol.sCost = ySol(idxVars.sCost);
sol.sRed = ySol(idxVars.sRed); 
sol.sAbsErr = ySol(idxVars.sAbsErr);
sol.exploitTR = max(0,min(ySol(idxVars.auxSAbsErr(2)),1));
sol.slack = ySol(idxVars.slack);

% update penalty weight
lambda = [full(res.lam_g); full(res.lam_x)];
sol.penaltyWeight = min([obj.penaltyWeight, ...
    10*max(abs(lambda))]);

% update cost set and compute optimal cost value
pZ_sCostOpt = polyZonotope(zeros(length(obj.id.cost),1), ...
        diag(ySol(idxVars.sCost)),[], ...
        eye(length(obj.id.cost)),obj.id.cost);
sol.costSet = subs(obj.costSet,pZ_sCostOpt,obj.id.cost);
fVal = volume(zonotope(sol.costSet));

end


%% auxiliary functions --------------------------------------------------

function termReg_po = preprocessCandTermReg(termReg,id_var_scaling,id_var_scalar)
% Wrapper for computing a parameterized partial zonotope over-approximation
% of the terminal region. Since the parameterization is linear in the
% optimization variables, we do not need to setup nonlinear function 
% handles for the generators (as opposed to the controller or the reachable
% set) to formulate the containment constraints.
%
% ------------------------------------------------------------------------

% compute partial zonotope over-approximation
[termReg_po] = computeTermRegNonlinSysPolyApproach.paramPartZonotope( ...
    termReg,id_var_scaling,id_var_scalar);

% recover generator matrix
termReg_po.G = cell2mat(cellfun(@(x) x.c,termReg_po.G, ...
    'UniformOutput',false));

% sort generator matrix
[~,idx] = sort(termReg_po.E'*termReg_po.idv);
termReg_po.G = termReg_po.G(:,idx);
termReg_po.E = termReg_po.E(:,idx);
termReg_po.isConstGen = termReg_po.isConstGen(idx);
termReg_po.cVar = termReg_po.cVar(idx);

% split center in static and variable part
termReg_po.cVar = termReg_po.c;
termReg_po.c = termReg_po.cVar.c;
termReg_po.cVar = termReg_po.cVar + zonotope(-termReg_po.c);

end

function [partZono,partZono_funHan] = computeGeneratorHandles(partZono,id_opt,id_var_scaling)
% This function prepares the center and generators of a given partial 
% zonotope over-approximation for optimization (restore full exponent
% matrix and account for translation of center due to factors with
% exclusively even exponents). Moreover, suitable function handles to
% evalute the constraint functions, etc. are computed.
%
% ------------------------------------------------------------------------

    function [pZ,pZ_han] = pZfunction(pZ,expMat_v,idv,id_opt,varargin)
        if isempty(varargin)
            is_cVar = false;
        else
            is_cVar = varargin{1};
        end

        % restore full exponent matrix + id vector 
        % (only consider optimization variables)
        if all(expMat_v == 0)
            tmp_expMat = [pZ.E;repmat(expMat_v,1,size(pZ.E,2))];
            pZ = polyZonotope(pZ.c,pZ.G,pZ.GI,tmp_expMat,vertcat(pZ.id,idv));
        else
            tmp_expMat = [zeros(size(pZ.E,1),1), pZ.E; ...
                repmat(expMat_v,1,1+size(pZ.E,2))];
            pZ = polyZonotope(zeros(size(pZ.c)),[pZ.c,pZ.G],pZ.GI, ...
                tmp_expMat,vertcat(pZ.id,idv));
        end

        % scale generators that shift the center 
        % (also non-opt.-variable ids to be considered)
        if is_cVar
            pZ = 0.5*pZ;
        end

        % create funtion handle
        pZ_han = @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(pZ,x,id_opt);
    end

% find ids of scaling variables (id_var_scaling) in scaling ids of partZono
% idv_opt = setdiff(id_opt,opts.id.pc_var,'stable');
[idv_com,~,tmp_idx] = intersect(id_var_scaling,partZono.idv,'stable');
if length(idv_com) ~= length(id_var_scaling)
    error('Invalid length of idv in part. zonotope over-approximation.')
end

partZono_funHan.c = cell(1);
partZono_funHan.G = cell(1,length(partZono.G));
partZono_funHan.cVar = partZono.cVar;

% initialise center
[tmp_c,~] = pZfunction(partZono.c,zeros(length(tmp_idx),1),id_var_scaling,id_opt);

% loop over generators
for ii = 1:length(partZono.G)
    [partZono.G{ii},partZono_funHan.G{ii}] = pZfunction( ...
        partZono.G{ii},partZono.E(tmp_idx,ii),id_var_scaling,id_opt,partZono.cVar(ii));
    if partZono.cVar(ii)
        tmp_c = exactPlus(tmp_c,partZono.G{ii});
    end
end

% create function handle and compute gradient for the center
partZono.c = tmp_c;
partZono_funHan.c = @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(tmp_c,x,id_opt);

% remove redundant fields from partZono
partZono = rmfield(partZono,'E');
partZono = rmfield(partZono,'isConstGen');
partZono = rmfield(partZono,'idv');

end


function [mPR,mPR_funHan] = computeParamReductionHandles(mapsReduce,id_opt,id_var_scaling)
% Wrapper of the auxiliary function computeGeneratorHandles for
% pre-processing of the parameterized zonotope order reduction maps.
%
% ------------------------------------------------------------------------

% pre-process mapsReduce: adapt ids to opt. variables
% - > generators that only have zero exponents in id_opt are treated as
% independent
[~,idx_idv_opt,idx_idv] = intersect(id_var_scaling,mapsReduce.idv,'stable');
tmp_expMat = zeros(length(id_var_scaling),size(mapsReduce.E,2));
tmp_expMat(idx_idv_opt,:) = mapsReduce.E(idx_idv,:);
idxRest = find(sum(tmp_expMat,1)==0);
if any(~mapsReduce.isConstGen(idxRest))
    error('Generators depending on p, c must not be considered as independent!')
end
if ~isempty(idxRest)
    mapsReduce.GI = mapsReduce.G(idxRest);
    mapsReduce.G(idxRest) = [];
    tmp_expMat(:,idxRest) = [];
    mapsReduce.cVar(idxRest) = [];
    mapsReduce.isConstGen(idxRest) = [];
end
mapsReduce.E = tmp_expMat;
mapsReduce.idv = id_var_scaling;

% split maps representing parameterized reduction into const. and
% non-const. (w.r.t. p, cTR)
idxConst = find(mapsReduce.isConstGen);
idxVar = find(~mapsReduce.isConstGen);

% process constant part (not depending on p, cTR) -------------------------

% pZ-representation of constant part (req. for computation of Hessian)
if isfield(mapsReduce,'Grest')
    tmp_Grest = sum(cell2mat(cellfun(@(x) abs(x.c),mapsReduce.GI,'UniformOutput',false)),2);
else
    tmp_Grest = zeros(mapsReduce.dim,1);
end
tmp_G = cell2mat(cellfun(@(x) abs(x.c),mapsReduce.G(idxConst),'UniformOutput',false));
% constant part of constraint modeled via center of pZ-rep.
mPR.map = polyZonotope(tmp_Grest,tmp_G,zeros(mapsReduce.dim,0), ...
    [zeros(length(id_opt)-length(id_var_scaling),length(idxConst));mapsReduce.E(:,idxConst)],id_opt);
mPR.id_red = mapsReduce.id;
% function handle
mPR_funHan.map = @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(mPR.map,x,id_opt);

% process non-constant part (depending on p, cTR) ------------------------

% extract non-constant part
mPR_var.c = mapsReduce.c;
mPR_var.G = mapsReduce.G(idxVar);
mPR_var.E = mapsReduce.E(:,idxVar);
mPR_var.idv = mapsReduce.idv;
mPR_var.cVar = mapsReduce.cVar(idxVar);
mPR_var.isConstGen = false(1,length(mPR_var.G));

% compute function handle and gradient
[mPR.aux,mPR_funHan.aux] = ...
    computeGeneratorHandles(mPR_var,id_opt,id_var_scaling);

% copy initial value of scales to mPR, mPR_funHan
mPR.initScales = mapsReduce.initScales;
mPR_funHan.initScales = mapsReduce.initScales;


end


function costFun = setupNominalCost(solver,id)
% This function returns the handlde to the nominal cost function (zonotope
% volume of the terminal region/an innner approximation thereof).
%
% ------------------------------------------------------------------------

pZ_vol = polyZonotope(zeros(1));
E = eye(length(id.cost));
for ii = 1:length(solver.costCoefficients)
    pZ_vol = exactPlus(pZ_vol, ...
        polyZonotope(0,solver.costCoefficients(ii),[], ...
        sum(E(:,solver.costExponents(ii,:)),2),id.cost));
end

% log transformation -----------------------------------------------------

funInner = @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(pZ_vol,x,id.cost);

% function handle of log(g(x))
costFun = @(x) -(log(funInner(x)));

end


function cost = evalCost(y,costFunHandle,idxVars,penaltyWeight)
% This function evaluates the full cost function (including soft constraint
% penalties) at a given point in the space of optimization variables.
%
% ------------------------------------------------------------------------

% nominal cost
cost = costFunHandle(y(idxVars.sCost));
    
% soft constraint penalties
cost = cost + penaltyWeight*sum(y(idxVars.slack));

end

function [ceqStruct,cineqStruct] ...
    = setupNonlinearConstraints(R,pZctrl,termReg,Z_U,mapsRed, ...
    nGenR,nGenCtrl,nGenTermReg,nGenU,nGenReduce, ...
    id_opt,idxVars,nx,nu)
% Concept: Nonlnear constraints are written in the form 
% g(x) + Aineq*x + const <= 0, h(x) + Aeq*x + const = 0
% where g(x) and h(x) are represented as polynomial zonotopes. This
% function returns structs for the equality and inequality constraints
% storing the respective components of the constraint functions.
%
% ------------------------------------------------------------------------

    function tmp_Con = generatorCondition(tmp_Con, ...
            counter,G_in,G_out,nGenIn,nGenOut,idxGamma,n)
        % generator condition [1, Eq. (7a)]: 
        % G_in(var_z) - G_out*var_Gamma = 0
        % h(x) = Gin(var_z), Aeq*x = -G_out*var_Gamma, const = 0

        for jj = 1:nGenIn

            % nonlinear part - > "G_in(var_z)"
            tmp_Con.pZ = stack(tmp_Con.pZ, G_in{jj});

            % linear part -> "-G_out*Gamma"
            tmp_Con.A.row = [tmp_Con.A.row, ...
                repmat(counter+(1:n),1,nGenOut)];
            tmp_Con.A.col = [tmp_Con.A.col, ...
                repelem(idxGamma(:,jj)',1,n)];
            tmp_Con.A.val = [tmp_Con.A.val, -G_out(:)'];
       
            % raise counter
            counter = counter + n;
        end
    end

    function tmp_Con = centerCondition(tmp_Con, ....
            counter,c_in,c_out,G_out,nGenOut,idxBeta,n)
        % center condition [1, Eq. (7b)]: 
        % c_out - c_in(var_z) - G_out*var_gamma = 0
        % h(x) = -c_in(var_z), Aeq*x = - G_out*var_gamma, const = c_out

        % nonlinear part
        tmp_Con.pZ = stack(tmp_Con.pZ, (-1)*c_in);
        
        % linear part 
        tmp_Con.A.row = [tmp_Con.A.row, ...
            repmat(counter+(1:n),1,nGenOut)];
        tmp_Con.A.col = [tmp_Con.A.col, ...
            repelem(idxBeta,1,n)];
        tmp_Con.A.val = [tmp_Con.A.val, ...
            -G_out(:)'];

        % bounds
        tmp_Con.const.row = [tmp_Con.const.row, ...
            counter + (1:n)];
        tmp_Con.const.val = [tmp_Con.const.val, ...
            c_out'];
    end

% equality constraints ---------------------------------------------------
% h(x) + Aeq*x + const = 0
% where h(x) is represented as polynomial zonotopes

% initialize constraint struct
ceqStruct.pZ = restoreId(polyZonotope.empty(),id_opt);
ceqStruct.A.row = [];
ceqStruct.A.col = [];
ceqStruct.A.val = [];
ceqStruct.const.row = [];
ceqStruct.const.val = [];

count_constr = 0;

% invariance constraint [1, Eq. (19b)], encoding [1, Eq. (7)]

% ... generator condition
idxGammaInv = reshape(idxVars.GammaInv,nGenTermReg,[]);
idxGammaInv = idxGammaInv(:,1:nGenR); % tightening
ceqStruct = generatorCondition(ceqStruct,count_constr, ...
    R.G,termReg.G,nGenR,nGenTermReg,idxGammaInv,nx);

count_constr = count_constr + nx*nGenR;

% ... center condition
ceqStruct = centerCondition(ceqStruct,count_constr, ...
    R.c,termReg.c,termReg.G,nGenTermReg,idxVars.gammaInv,nx);
% ... account for variable center of terminal region
ceqStruct.A.row = [ceqStruct.A.row, ...
    count_constr+(1:nx)];
ceqStruct.A.col = [ceqStruct.A.col, ...
    idxVars.delta_c];
ceqStruct.A.val = [ceqStruct.A.val, ...
    ones(1,nx)];

count_constr = count_constr + nx;

% input constraint [1, Eq. (19d)], encoding [1, Eq. (7)]

% ... generator condition
idxGammaU = reshape(idxVars.GammaU,nGenU,[]);
ceqStruct = generatorCondition(ceqStruct,count_constr, ....
    pZctrl.G,Z_U.G,nGenCtrl,nGenU,idxGammaU,nu);

count_constr = count_constr + nu*nGenCtrl;

% ... center condition 
ceqStruct = centerCondition(ceqStruct,count_constr, ...
    pZctrl.c,Z_U.c,Z_U.G,nGenU,idxVars.gammaU,nu);

count_constr = count_constr + nu;

% upper bound on scaling factors for reducing generators 
% h(x) = mapsRed.map(var_z) -> generators that are only scaled
% Aeq*x = sum(auxSRed,2) - mapsRed.initScales.*sRed -> add auxiliary
% scaling factors (auxSRed) required for resolving absolute values; sum
% has to be equal to the product of the initial length and the scaling 
% factors of the reducing generators
% const = 0

% ... nonlinear part
ceqStruct.pZ = stack(ceqStruct.pZ, mapsRed.map);

% ... linear part - scaling factors of reducing generators
ceqStruct.A.row = [ceqStruct.A.row, ...
    count_constr + (1:idxVars.n_sRed)];
ceqStruct.A.col = [ceqStruct.A.col, ...
    idxVars.sRed];
ceqStruct.A.val = [ceqStruct.A.val, ...
    -mapsRed.initScales'];  %-ones(1,idxVars.n_sRed)];
% ... linear part - auxiliary scaling factors
ceqStruct.A.row = [ceqStruct.A.row, ...
    repmat(count_constr+(1:idxVars.n_sRed),1,idxVars.n_auxSRed)];
ceqStruct.A.col = [ceqStruct.A.col, ...
    repelem(idxVars.auxSRed,1,idxVars.n_sRed)];
tmp_val = repmat(eye(idxVars.n_sRed),1,nGenReduce);
ceqStruct.A.val = [ceqStruct.A.val, ...
    tmp_val(:)'];

count_constr = count_constr + idxVars.n_sRed;

% finalize nonlinear equality constraints --------------------------------
ceqStruct.nCon = count_constr; 

% remove zero entries from linear part and create Aeq, const
zero_idx = find(ceqStruct.A.val == 0);
ceqStruct.A.val(zero_idx) = [];
ceqStruct.A.row(zero_idx) = [];
ceqStruct.A.col(zero_idx) = [];

ceqStruct.A.mat = sparse(ceqStruct.A.row,ceqStruct.A.col,ceqStruct.A.val, ...
    ceqStruct.nCon,idxVars.nVars);

% compute function handle of nonlinear terms
ceqStruct.nonlin.handle = ...
    @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(ceqStruct.pZ,x,id_opt);

% inequality constraints -------------------------------------------------
% g(x) + Aineq*x + const <= 0
% where g(x) is represented as polynomial zonotopes

% initialize constraint struct
cineqStruct.pZ = restoreId(polyZonotope.empty(),id_opt);

cineqStruct.A.row = [];
cineqStruct.A.col = [];
cineqStruct.A.val = [];
cineqStruct.const.row = [];
cineqStruct.const.val = [];

count_constr = 0;

% parameterized reduction 
% ... resolve absolute values: abs(G{ii}(x)) <= auxSRed
idx_auxSRed = reshape(idxVars.auxSRed,idxVars.n_sRed,nGenReduce);

for ii = 1:nGenReduce
    for kk = 1:2
            % g(x) = +-mapsReduce.aux.G(var_z), 
            % Aineq*x = -auxSRed, 
            % const = 0
    
            % nonlinear part
            cineqStruct.pZ = stack(cineqStruct.pZ, ...
                (-1)^kk*mapsRed.aux.G{ii});
            
            % linear part
            cineqStruct.A.row = [cineqStruct.A.row, ...
                count_constr+(1:idxVars.n_sRed)];
            cineqStruct.A.col = [cineqStruct.A.col, ...
                idx_auxSRed(:,ii)'];
            cineqStruct.A.val = [cineqStruct.A.val, ...
                -ones(1,idxVars.n_sRed)];           

            count_constr = count_constr + idxVars.n_sRed;
    end
end

% finalize nonlinear inequality constraints ------------------------------
cineqStruct.nCon = count_constr;

% remove zero entries from linear part and create Aineq, const
zero_idx = find(cineqStruct.A.val == 0);
cineqStruct.A.val(zero_idx) = [];
cineqStruct.A.row(zero_idx) = [];
cineqStruct.A.col(zero_idx) = [];

cineqStruct.A.mat = sparse(cineqStruct.A.row,cineqStruct.A.col, ...
    cineqStruct.A.val,cineqStruct.nCon,idxVars.nVars);

% compute function handle of nonlinear terms
cineqStruct.nonlin.handle = ...
    @(x) computeTermRegNonlinSysPolyApproach.evalPZfuncCasadi(cineqStruct.pZ,x,id_opt);

end

function [Aineq,bineq,Aeq,beq] = linearConstraints(solver,termReg, ...
    idxVars,nGenR,nGenTermReg,nGenCtrl,nGenU,R_GI)
% This function computes the (sparse) matrices modelling the linear 
% inequality constraints Aineq <= bineq
% The following constraints are modelled as linear constraints:
%       - invariance constraint:    norm condition [1, Eq. (7c)]
%       - input constraint:         norm condition [1, Eq. (7c)]
%       - state constraint:         terminal region contained in the set of 
%                                   admissible states - see [1, Eq. (19c)]
%       - simplified cost:          reduced-order cost set is contained in
%                                   terminal region - encoding [1, Eq. (7)]
%       - invariance constraint:    independent generators [1, Eq. (7a)]
%
% ------------------------------------------------------------------------

% inequality constraints -------------------------------------------------

Aineq.row = []; bineq.row = [];
Aineq.col = []; 
Aineq.val = []; bineq.val = [];
count_constr = 0;

% invariance constraint: norm([Gamma,gamma],'inf') <= scales + slack
% see [1, Eq. (7c)]
idx_auxGammaInv = reshape(idxVars.auxGammaInv,nGenTermReg,[]);
nGenIn = size(idx_auxGammaInv,2);

% ... sum of auxiliary variables (abs. of Gamma and gamma) <= scale + slack
for ii = 1:nGenTermReg  
    % set indices + values of sparse matrix
    % -> s, nGenRin entries of auxGammaInv, slack
    Aineq.row = [Aineq.row, ...
        (count_constr+1)*(ones(1,nGenIn+3))]; 
    Aineq.col = [Aineq.col, ...
        idxVars.s(ii),idx_auxGammaInv(ii,:),idxVars.auxBetaInv(ii),idxVars.slack];
    Aineq.val = [Aineq.val, ...
        -1, ones(1,nGenIn),1,-1];

    % raise counter
    count_constr = count_constr + 1;
end

% ... resolve absolute values (Gamma)
for ii = 1:idxVars.n_GammaInv
    % absolute value of entries of GammaInv
    Aineq.row = [Aineq.row, ...
        count_constr+1, count_constr+1, ...
        count_constr+2, count_constr+2];
    Aineq.col = [Aineq.col, ...
        idxVars.GammaInv(ii), idxVars.auxGammaInv(ii), ...
        idxVars.GammaInv(ii), idxVars.auxGammaInv(ii)];
    Aineq.val = [Aineq.val, 1, -1,-1,-1];

    % raise counter
    count_constr = count_constr + 2;
end

% ... resolve absolute values (gamma)
for ii = 1:idxVars.n_gammaInv
    % absolute value of entries of gammaInv
    Aineq.row = [Aineq.row, ...
        count_constr+1, count_constr+1, ...
        count_constr+2, count_constr+2];
    Aineq.col = [Aineq.col, ...
        idxVars.gammaInv(ii), idxVars.auxBetaInv(ii), ...
        idxVars.gammaInv(ii), idxVars.auxBetaInv(ii)];
    Aineq.val = [Aineq.val, 1, -1,-1,-1];

    % raise counter
    count_constr = count_constr + 2;
end

% input constraint: norm([GammaU,gammaU],'inf') <= 1
% see [1, Eq. (7c)]

% ... sum of auxiliary variables (abs. of Gamma and gamma) <= scale + slack
for ii = 1:nGenU
    % vectorized indices of ii-th row of GammaU
    tmp_idx = ...
        idxVars.auxGammaU(ii):nGenU:idxVars.auxGammaU(ii)+(nGenCtrl-1)*nGenU;
    
    % set indices + values of sparse matrix
    % -> nGenRin entries of GammaU
    Aineq.row = [Aineq.row, ...
        (count_constr+1)*(ones(1,nGenCtrl+1))]; 
    Aineq.col = [Aineq.col, ...
        tmp_idx,idxVars.auxBetaU(ii)];
    Aineq.val = [Aineq.val, ...
        ones(1,nGenCtrl),1];

    % set indices + values of right-hand side
    bineq.row = [bineq.row, ...
        count_constr+1]; 
    bineq.val = [bineq.val, ...
        1];

    % raise counter
    count_constr = count_constr + 1;
end

% ... resolve absolute values (GammaU)
for ii = 1:idxVars.n_GammaU
    % absolute value of entries of GammaU
    Aineq.row = [Aineq.row, ...
        count_constr+1, count_constr+1, ...
        count_constr+2, count_constr+2];
    Aineq.col = [Aineq.col, ...
        idxVars.GammaU(ii), idxVars.auxGammaU(ii), ...
        idxVars.GammaU(ii), idxVars.auxGammaU(ii)];
    Aineq.val = [Aineq.val, 1, -1,-1,-1];

    % raise counter
    count_constr = count_constr + 2;
end

% ... resolve absolute values (gammaU)
for ii = 1:idxVars.n_gammaU
    % absolute value of entries of gammaU
    Aineq.row = [Aineq.row, ...
        count_constr+1, count_constr+1, ...
        count_constr+2, count_constr+2];
    Aineq.col = [Aineq.col, ...
        idxVars.gammaU(ii), idxVars.auxBetaU(ii), ...
        idxVars.gammaU(ii), idxVars.auxBetaU(ii)];
    Aineq.val = [Aineq.val, 1,-1,-1,-1];

    % raise counter
    count_constr = count_constr + 2;
end

% state constraint [1, Eq. (19c)]
% termReg \subseteq \X

nH = size(solver.H_X_con.b,1);
% ... variable part of center: H*\Delta c
Aineq.row = [Aineq.row, repmat(count_constr+(1:nH),1,idxVars.n_dc)];
Aineq.col = [Aineq.col, repelem(idxVars.delta_c,1,nH)];
Aineq.val = [Aineq.val, solver.H_X_con.A(:)'];
% ... generators: abs(H*termReg.G)*s
Aineq.row = [Aineq.row, repmat(count_constr+(1:nH),1,nGenTermReg)];
Aineq.col = [Aineq.col, repelem(idxVars.s,1,nH)];
tmp = abs(solver.H_X_con.A*termReg.G);
Aineq.val = [Aineq.val, tmp(:)'];
% ... right-hand side h - H*termReg.c
bineq.row = [bineq.row, count_constr+(1:nH)];
tmp = solver.H_X_con.b-solver.H_X_con.A*termReg.c;
bineq.val = [bineq.val, tmp(:)'];

count_constr = count_constr + nH;

% reduced cost set containment problem
% ... reduced cost set must be contained in terminal region
% -> encoding [1, Eq. (7)]

if ~solver.flagCostExact
    % norm condition [1, Eq. (7c)]
    % sum of aux. vars(j-th row) <= scale(j)
    idx_auxGammaCost = reshape(idxVars.auxGammaCost,nGenTermReg,[]);
    nGenIn = size(solver.costSet.G,2);
    for ii = 1:nGenTermReg
        % set indices + values of sparse matrix
        % -> s, nGenIn entries of auxGammaCost
        Aineq.row = [Aineq.row, (count_constr+1)*(ones(1,nGenIn+1))]; 
        Aineq.col = [Aineq.col, ...
            idxVars.s(ii), ...
            idx_auxGammaCost(ii,:)];
        Aineq.val = [Aineq.val, -1, ones(1,nGenIn)];
    
        % raise counter
        count_constr = count_constr + 1;
    end
    
    % resolve absolute values (Gamma)
    for ii = 1:idxVars.n_auxGammaCost
        % absolute value of entries of GammaCost
        Aineq.row = [Aineq.row, ...
            count_constr+1, count_constr+1, ...
            count_constr+2, count_constr+2];
        Aineq.col = [Aineq.col, ...
            idxVars.GammaCost(ii), idxVars.auxGammaCost(ii), ...
            idxVars.GammaCost(ii), idxVars.auxGammaCost(ii)];
        Aineq.val = [Aineq.val, 1, -1,-1,-1];
    
        % raise counter
        count_constr = count_constr + 2;
    end
end

% finalize linear inequality constraints ---------------------------------

% remove zero entries
zero_idx = find(Aineq.val == 0);
Aineq.val(zero_idx) = [];
Aineq.row(zero_idx) = [];
Aineq.col(zero_idx) = [];

% assign output arguments
Aineq.nCon = count_constr;
Aineq.mat = sparse(Aineq.row,Aineq.col,Aineq.val,Aineq.nCon,idxVars.nVars);
bineq.nCon = count_constr;

% equality constraint ----------------------------------------------------

Aeq.row = [];
Aeq.col = []; 
Aeq.val = []; 
beq.row = [];
beq.val = [];
count_constr = 0;

% invariance constraint: independent generators (including tightening)
% ... generator condition [1, Eq. (7a)]
idx_GammaInv_GI = reshape( ...
    idxVars.GammaInv(nGenR*nGenTermReg+1:end),nGenTermReg,[]);
for ii = 1:size(R_GI,2)
    Aeq.row = [Aeq.row,repmat(count_constr+(1:solver.nx),1,nGenTermReg)];
    Aeq.col = [Aeq.col,repelem(idx_GammaInv_GI(:,ii)',1,solver.nx)];
    Aeq.val = [Aeq.val, termReg.G(:)'];
    beq.row = [beq.row,count_constr+(1:solver.nx)];
    beq.val = [beq.val,R_GI(:,ii)'];
    count_constr = count_constr + solver.nx;
end

% scaling factors of reduced cost set
% ... generator condition [1, Eq. (7a)]
if ~solver.flagCostExact
    Gout = termReg.G;
    Gin = solver.costSet.G;

    idx_GammaCost = reshape(idxVars.GammaCost,nGenTermReg,[]);
    for ii = 1:size(Gin,2)
        Aeq.row =  [Aeq.row,repmat(count_constr+(1:solver.nx), ...
            1,size(Gout,2)+1)];
        Aeq.col = [Aeq.col, repelem(idxVars.sCost(ii),1,solver.nx), ...
            repelem(idx_GammaCost(:,ii)',1,solver.nx)];
        Aeq.val = [Aeq.val, -Gin(:,ii)',Gout(:)'];

        count_constr = count_constr + solver.nx;
    end
end

% finalize linear equality constraints -----------------------------------

% remove zero entries
zero_idx = find(Aeq.val == 0);
Aeq.val(zero_idx) = [];
Aeq.row(zero_idx) = [];
Aeq.col(zero_idx) = [];

% assign output arguments
Aeq.nCon = count_constr;
Aeq.mat = sparse(Aeq.row,Aeq.col,Aeq.val,Aeq.nCon,idxVars.nVars);
beq.nCon = count_constr;

end


function [lb,ub,idxCon] = mergeConstraints( ...
    ceqStruct,cineqStruct,Aeq,beq,Aineq,bineq,absErr_con,absErr_conAux)
% This function concatenates the lower and upper bounds of all equality and
% inequality constraints (see the CasADi nonlinear programming docs):
%   -inf <= g(x) + Aineq*x + const <= 0
%   0 <= h(x) + Aeq*x + const = 0
% % Constraint stack:
% nonlinear equality constraints (ceqStruct), nonlinear inequality 
% constraints (cineqStruct), linear equality constraints (Aeq), linear
% inequality constraints (Aineq), parameterized abstraction error
% (absErr_con), auxiliary constraints for parameterized abstraction error
% (absErr_conAux)
% ------------------------------------------------------------------------

lb = [];
ub = [];

idxCon.eqCon = [];
idxCon.ineqCon = [];

% nonlinear equality constraints (ceq)
tmp_b = zeros(ceqStruct.nCon,1);
tmp_b(ceqStruct.const.row) = -ceqStruct.const.val;
lb = [lb; tmp_b]; ub = [ub; tmp_b]; 
idxCon.eqCon = [idxCon.eqCon, 1:ceqStruct.nCon];
nCon = ceqStruct.nCon;

% nonlinear inequality constraints (cineq)
tmp_ub = zeros(cineqStruct.nCon,1);
tmp_ub(cineqStruct.const.row) = -cineqStruct.const.val;
lb = [lb; -inf(cineqStruct.nCon,1)]; ub = [ub; tmp_ub];
idxCon.ineqCon = [idxCon.ineqCon, nCon + (1:cineqStruct.nCon)];
nCon = nCon + cineqStruct.nCon;

% linear equality constraints (Aeq)
tmp_b = zeros(Aeq.nCon,1);
tmp_b(beq.row) = beq.val;
lb = [lb; tmp_b]; ub = [ub; tmp_b];
idxCon.eqCon = [idxCon.eqCon, nCon + (1:Aeq.nCon)];
nCon = nCon + Aeq.nCon;

% linear inequality constraints (Aineq)
tmp_ub = zeros(Aineq.nCon,1);
tmp_ub(bineq.row) = bineq.val;
lb = [lb; -inf(Aineq.nCon,1)]; ub = [ub; tmp_ub];
idxCon.ineqCon = [idxCon.ineqCon, nCon + (1:Aineq.nCon)];
nCon = nCon + Aineq.nCon;

% parameterized abstraction error
% ... abstraction error approximation
lb = [lb; -inf(absErr_con.nCon,1)];
ub = [ub; absErr_con.bineq];
idxCon.ineqCon = [idxCon.ineqCon, nCon+(1:absErr_con.nCon)];
nCon = nCon + absErr_con.nCon;
% ... auxiliary constraints
lb = [lb; -inf(absErr_conAux.nCon,1)];
ub = [ub; absErr_conAux.bineq];
idxCon.ineqCon = [idxCon.ineqCon, nCon+(1:absErr_conAux.nCon)];
nCon = nCon + absErr_conAux.nCon;

idxCon.nCon = nCon;

end


function con = evalConstraints(y,ceqStruct,cineqStruct,Aeq,Aineq, ...
    absErr_con,absErr_conAux,idxVars)
% This function returns the values of the constraint function at a given
% point in the space of optimization variables.
% Constraint stack:
% nonlinear equality constraints (ceqStruct), nonlinear inequality 
% constraints (cineqStruct), linear equality constraints (Aeq), linear
% inequality constraints (Aineq), parameterized abstraction error
% (absErr_con), auxiliary constraints for parameterized abstraction error
% (absErr_conAux)
%
% ------------------------------------------------------------------------

% extract arguments of the nonlinear constraints
var_z = y(idxVars.z);

% nonlinear equality constraint
con = ceqStruct.A.mat*y;
if ~(ceqStruct.pZ.dim == 0)
    con = ceqStruct.nonlin.handle(var_z) + ceqStruct.A.mat*y;
end

% nonlinear inequality constraint
tmp_con = cineqStruct.A.mat*y;
if ~(cineqStruct.pZ.dim == 0)
    tmp_con = tmp_con + cineqStruct.nonlin.handle(var_z);
end
con = [con; tmp_con];

% linear equality constraint
con = [con; Aeq.mat*y];

% linear inequality constraint
con = [con; Aineq.mat*y];

% parameterized abstraction error
con = [con; absErr_con.Aineq*y];
con = [con; absErr_conAux.Aineq*y];

end

function y0 = computePrimalInitialGuess(solver,termReg,R,R_GI, ...
    pZu_vp,mapsParamReduce,nGenReduce,p0,idxVars)
% This function computes an initial guess for the primal optimization
% variables.
%
% ------------------------------------------------------------------------

y0 = zeros(idxVars.nVars,1);

% parameters of reachable set 
% s = 1, sRed = 1, sAbsErr = 1, p given
y0(idxVars.p) = p0;
y0(idxVars.delta_c) = 0;
y0(idxVars.s) = 1;
y0(idxVars.sRed) = 1; %mapsParamReduce.initScales;  %1;
y0(idxVars.sRedConst) = 1;
% y0(idxVars.sBBox) = 1;
y0(idxVars.sAbsErr) = 1;
y0(idxVars.auxSAbsErr) = 1;
y0(idxVars.sCost) = 1;
% y0(idxVars.cRad) = termReg.c-xExp;
y0(idxVars.auxSAbsErr(2)) = 0;
z0 = y0(idxVars.z);  

% invariance constraint 
% ... GammaInv, gammaInv: check containment of R in termReg for z0
Gin = cellfun(@(x) x(z0),R.G,'UniformOutput',false);
Gin = cell2mat(reshape(Gin,1,length(R.G)));
cIn = R.c(z0);

% ... solve LP
% Gtight = eye(opts.nx)*opts.optim.constraintTolerance;
GR = [Gin,R_GI];
[~,contVars] = zonoContainment( ...
    zonotope(cIn,GR), ...
    zonotope(termReg.c,termReg.G), ...
    solver.constraintTolerance);
y0(idxVars.GammaInv) = contVars.Gamma(:);
y0(idxVars.gammaInv) = contVars.gamma;
y0(idxVars.slack) = contVars.slack;

% ... aux variables: -> take absolute value of GammaInv, gammaInv
y0(idxVars.auxGammaInv) = abs(y0(idxVars.GammaInv));
y0(idxVars.auxBetaInv) = abs(y0(idxVars.gammaInv));

% input constraint
% ... GammaU, gammaU: check containment of pZu_vp in Z_U for z0
Gin = cellfun(@(x) x(z0),pZu_vp.G,'UniformOutput',false);
Gin = cell2mat(reshape(Gin,1,length(pZu_vp.G)));
cIn = pZu_vp.c(z0);

% ... solve LP
[~,contVars] = zonoContainment(zonotope(cIn,Gin),solver.Z_U,solver.constraintTolerance);
y0(idxVars.GammaU) = contVars.Gamma(:);
y0(idxVars.gammaU) = contVars.gamma;

% ... aux variables: -> take absolute value of GammaU, gammaU
y0(idxVars.auxGammaU) = abs(y0(idxVars.GammaU));
y0(idxVars.auxBetaU) = abs(y0(idxVars.gammaU));

% auxiliary variables for parameterized reduction ------------------------
if nGenReduce > 0
    Gvar = cellfun(@(x) x(z0),mapsParamReduce.aux.G,'UniformOutput',false);
    Gvar = reshape(cell2mat(reshape(Gvar,[],nGenReduce)),[],1);
    y0(idxVars.auxSRed) = abs(Gvar);
end

% reduced cost set
if ~solver.flagCostExact
    [~,contVars] = zonoContainment( ...
        zonotope(zeros(solver.nx,1),solver.costSet.G), ...
        zonotope(zeros(solver.nx,1),termReg.G), ...
        solver.constraintTolerance);
    y0(idxVars.GammaCost) = contVars.Gamma(:);
    y0(idxVars.auxGammaCost) = abs(contVars.Gamma(:));
end

end
