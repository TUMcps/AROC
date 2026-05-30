function [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible,varargin)
%SOLVEOPTPROBLEM - models and solves the polynomial programming 
%       approximation of the terminal region computation problem.
%
% Syntax:
%       [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible)
%       [obj,termReg_opt,auxData_iter] = solveOptProblem(obj,dynamics,termReg,flagInitGuessFeasible,bloatAbsErr)
%
% Description:
%       This function models and solves the polynomial programming 
%       approximation in [1, Eq. (19)] of the optimization problem in 
%       [1, Eq. (6)], which returns a robust control invariant set of 
%       a given nonlinear dynamical system with maximum volume.  
%
% Input Arguments:
%       -obj:                   instance of class
%                               computeTermRegNonlinSysPolyApproach
%       -dynamics:              instance of class termRegNonlinSysDynamics                        
%       -termReg:               current candidate for the terminal region
%                               (class: termRegNonlinSysPolyApproach)
%       -flagInitGuessFeasible: indicates whether termReg admits a feasible
%                               initial guess (or not)
%                               (boolean)
%       -bloatAbsErr:           (optional) scaling factor for enlarging the 
%                               set of abstraction errors (see the 
%                               verification procedure described in 
%                               [2, Sec. III 2])
%                               (bloatAbsErr >= 1)
%
% Output Arguments:
%       -obj:               updated instance of class
%                           computeTermRegNonlinSysPolyApproach
%       -termReg_opt:       updated candidate for the terminal region
%                           (class: termRegNonlinSysPolyApproach)
%       -auxData_iter:      additional solution data (e.g. flag indicating
%                           feasibility)
%                           (class: termRegNonlinSysSolutionAuxData)
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
%       [2] L. Schäfer et al., "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770.
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

% parameterized reachability analysis ------------------------------------

% parameterize center of terminal region (translation)
pZ_cTR = polyZonotope(zeros(obj.nx,1),eye(obj.nx),[], ...
    eye(obj.nx),obj.id.delta_c );
termRegSet_sc = exactPlus(termReg.set,pZ_cTR);

% reachability analysis
[approx_R,mapsParamReduce] = obj.paramReach( ...
    dynamics,termRegSet_sc,obj.pZu_vp,termReg.ctrlParam,bloatAbsErr,flagInitGuessFeasible);

% paramterized approximations (of lower bound) of abstraction error
paramAbsErr_coefficients = obj.approxAbsErrPwLin_coefficients(dynamics,termReg.set);

% solve optimization problem ---------------------------------------------

[sol,~] = obj.optimizeRCIpoly(termRegSet_sc,approx_R,mapsParamReduce, ...
    paramAbsErr_coefficients,termReg.ctrlParam);

% post-process solution --------------------------------------------------

% evaluate abstraction error approximation
width_estAbsErr = obj.approxAbsErrPwLin_eval( ...
    termRegSet_sc,sol.s,sol.delta_c,paramAbsErr_coefficients);
estAbsErr  = interval(-width_estAbsErr,width_estAbsErr);

% check feasibility of invariance constraint
% -> update candidate terminal region - translate center
termRegSet_s = resolve(termRegSet_sc,sol.delta_c,obj.id.delta_c );
% -> template for scaling generators of cand. terminal region
pZ_sOpt = polyZonotope(zeros(length(obj.id.termReg),1),diag(sol.s),[], ...
    eye(length(obj.id.termReg)),obj.id.termReg);
% -> update candidate terminal region
termRegSet_opt = subs(termRegSet_s,pZ_sOpt,obj.id.termReg);
% -> evaluate parameterized approxmiation of reachable set
approx_R = subs(approx_R,pZ_sOpt,obj.id.termReg);
pZ_redOpt = polyZonotope( ...
    zeros(length(mapsParamReduce.id),1),diag(sol.sRed), ...
    [],eye(length(mapsParamReduce.id)),mapsParamReduce.id);
approx_R = subs(approx_R,pZ_redOpt,mapsParamReduce.id);
approx_R = resolve(approx_R,[sol.delta_c;sol.p],[obj.id.delta_c ;obj.id.p]);
% -> check contaiment
if ~contains(zonotope(termRegSet_opt),zonotope(approx_R),'approx:st',1e-6)
    % if not contained: set slack variables to 1 for post-processing
    sol.slack = 1;
end

% post-processing
[obj,auxData_iter] = obj.postprocessIterate(sol.s,sol.sCost,sol.slack, ...
    sol.exploitTR,estAbsErr,sol.solverTime,sol.setupTime);

% assign output values ---------------------------------------------------

% controller
ctrl_opt = resolve(obj.pZu_vp,sol.p,obj.id.p);

% terminal region object
termReg_opt = termRegNonlinSysPolyApproach( ...
    dynamics.dynamicsHandle,dynamics.sysType,termRegSet_opt,ctrl_opt, ...
    sol.p,dynamics.dt,obj.H_U,obj.W);

end
