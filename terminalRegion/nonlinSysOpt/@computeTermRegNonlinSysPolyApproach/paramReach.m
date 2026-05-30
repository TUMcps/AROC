function [R_next,mapsReduce] = paramReach(obj,dynamics,candTermReg,ctrl_template,candP,bloatAbsErr,flagReduceFeasible)
%PARAMREACH - parameterized reachability analysis
%
% Syntax:
%       [R_next,mapsReduce] = paramReach(obj,dynamics,candTermReg,ctrl_template,candP,bloatAbsErr,flagReduceFeasible)
%
% Description:
%       This function computes the time-point reachable set in [1, Eq.
%       (17)]. This involves computing reachable sets of the abstraction
%       using the polynomialization algorithm, computation of the
%       abstraction error, and the parameterized order reduction to limit
%       the number of variables and constraints in [1, Eq. (19)].
%
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysPolyApproach
%       -dynamics:          instance of class termRegNonlinSysDynamics         
%       -candTermReg:       candidate terminal region 
%                           (class: polyZonotope)
%       -ctrl_template:     controller template
%                           (class: polyZonotope)
%       -candP:             candidate controller prarameters
%                           (double array of dimension
%                           [length(obj.id.p),1])
%       -bloatAbsErr:       scaling factor for enlarging the set of 
%                           abstraction errors (see the  verification 
%                           procedure described in [2, Sec. III 2])
%                           (bloatAbsErr >= 1)
%       -flagReduceFeasible: true if the currenct candidate solution is
%                           feasible, triggers adaptive order reduction,
%                           see method reduceTemplate
%
% Output Arguments:
%       - Rfin:         parameterized reachable set
%                       (class: polyZonotope)
%       - mapsReduce:   struct storing the ingredients for the
%                       parameterized order reduction, see method 
%                       reduceTemplate 
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/reduceTemplate
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control 
%           Invariant Sets of Nonlinear Systems Using Polynomial 
%           Controller Synthesis," American Control Conference, 2024, 
%           pp. 4162-4169. 
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

% initialization ---------------------------------------------------------

% augmented initial set
R0 = stack(candTermReg,ctrl_template);

% set parameters
tmp_params = dynamics.reachParams;
tmp_params.R0 = R0;

% approximate time-point reachable set -----------------------------------
    
% set order of reachable sets
tmp_reachOpts = computeReachSetOrder(R0,dynamics.paramReachOpts,obj.nx,obj.nu);
% set linearization point
tmp_reachOpts.linearizationPoint = [obj.xEq;obj.uEq];

% compute reachable set of the abstraction
tmp_R = reach(dynamics.sys,tmp_params,tmp_reachOpts);
if isempty(tmp_R)
    error('reach:setexplosion','reachable set explosion');
end

% project reachable set to augmented state-space
R_abs = project(tmp_R.timePoint.set{end},1:obj.nx); %+obj.nu);
% the reachable set of the abstraction due tmp_params.U is removed from Rdep
% -> no independent generators (dynamics.paramReachOpts.approxDepOnly=true)
assert(~isempty(R_abs.GI), 'Independent generators are assumed to be removed!')

% parameterized approximation of the abstraction error
pZ_absErr = paramAbsErr(obj,dynamics,candTermReg,bloatAbsErr);
% compute reachable set due to pZ_absErr
R_absErr = computeParticularSolution(dynamics,pZ_absErr,obj.id.idMaxAux);

% reachable set due to uncertain inputs
R_err = R_absErr + dynamics.RlinW;

% template-based order reduction (template is an interval)
[R_next,mapsReduce] = reduceTemplate(obj,R_abs,R_err,candTermReg,candP,flagReduceFeasible);

% assign output arguments
R_next = project(R_next,1:obj.nx);

end


%% auxiliary functions ---------------------------------------------------

function reachOpts = computeReachSetOrder(Rtp,reachOpts,nx,nu)
% Computation of suitable values for the zonotope orders used in
% reachability analysis to avoid reduction inside CoRA. Reduction of the
% approximation of the approximation Rtp of the time-point reachable sets
% is conducted after calling CoRA to retain dependencies despite reduction.
%
% ------------------------------------------------------------------------

nGen_R = size(Rtp.G,2);
order_R = ceil(nGen_R/(nx+nu));

% errorOrder: reduce time-point reachable set before quadMap
reachOpts.errorOrder = order_R; 
% intermediateOrder: reduce quadMap of time-point reachable set
reachOpts.intermediateOrder = ...
        ceil(0.5*nGen_R*(nGen_R+1)/(nx+nu));
% zonotopeOrder: reduction of result
reachOpts.zonotopeOrder = order_R + reachOpts.intermediateOrder;

end

function pZ_absErr = paramAbsErr(solver,dynamics,candTermReg,bloatAbsErr)
% Returns a polynomial zonotope representing the parameterized abstraction
% error evaluated at the current candidate terminal region.
%
% ------------------------------------------------------------------------

% plug in initial value of center shift
candTermReg = resolve(candTermReg,zeros(dynamics.nx,1),solver.id.delta_c );

% compute set of abstraction errors at candTermReg
absErr = solver.computeAbsErr(dynamics,candTermReg);

% set polynomial zonotope (in the augmented state space
c = zeros(dynamics.nx+dynamics.nu,1);
G = [diag(absErr.sup)*bloatAbsErr; zeros(dynamics.nu,dynamics.nx)];
pZ_absErr = polyZonotope(c,G,[], eye(dynamics.nx),solver.id.absErr);

end

function Rpart = computeParticularSolution(dynamics,varInput,idMaxAux)
% Computes an approximation of the particular solution for the set of
% time-varying inputs varInput.
%
% ------------------------------------------------------------------------

% compute center and generator matrix of reachable set
G_Rpart = cell(dynamics.nrTermsInputSolution,1);
c_Rpart = zeros(dynamics.nx,1);
for kk = 1:dynamics.nrTermsInputSolution
    G_Rpart{kk} = dynamics.Apower_dt_fact{kk}*varInput.G;
    c_Rpart = c_Rpart + dynamics.Apower_dt_fact{kk}*varInput.c;
end

% set exponent matrix and vector of ids
expMat_Rpart = [repmat(varInput.E,1,dynamics.nrTermsInputSolution); ...
    eye((dynamics.nrTermsInputSolution)*size(varInput.E,2))];
id_Rpart = [reshape(varInput.id,1,[]), idMaxAux + (1:size(expMat_Rpart,2))]';

% pZ-representation of Rvi
Rpart = polyZonotope(c_Rpart,cat(2,G_Rpart{:}),[], ...
    expMat_Rpart,id_Rpart);

end
