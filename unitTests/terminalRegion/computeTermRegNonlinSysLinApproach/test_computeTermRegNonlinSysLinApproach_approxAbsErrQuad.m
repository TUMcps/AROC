function res = test_computeTermRegNonlinSysLinApproach_approxAbsErrQuad
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_APPROXABSERRQUAD - unit test
%function of the quadratic approximation of the abstraction error.
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_approxAbsErrQuad
%
% Description:
%       This script tests whether the quadratic approximation of the 
%       abstraction error proposed in [1, Proposition 1] satisifes the 
%       condition in [1, Def. 3, C1)+C2)]. To this end, we generate random 
%       pairs of terminal regions (represented as a zonotope) and generator 
%       scaling factors for the cart benchmark.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/approxAbsErrQuad_coefficients, 
%       @computeTermRegNonlinSysLinApproach/approxAbsErrQuad_eval
% 
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
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

benchmark = 'cart';

% sets of admissible states, control inputs, and disturbances
width = [6; 6];
param.X = interval(-width, width);
width = 14;
param.U = zonotope(interval(-width,width));
width = [0.1;0.1];
W = interval(-width,width);
param.W = zonotope(W); 

% algorithm settings
opts.timeStep = 0.1;
opts.termRegOrder = 5;
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.sysType = termRegSysType.DT;
opts.integrator = discretizeDynamics.RK2;
opts.nonlinSysTermRegAlg = 'lin';
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% initialize dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% set trust region
solver.tr_set = zonotope(zeros(solver.nx,1),eye(solver.nx));

% interval enclosure of set of admissible inputs and disturbances
IHu = interval(param.U + (zonotope(-opts.uEq)));
du = max(abs(IHu.sup),abs(IHu.inf));
IHw = interval(param.W);
dw = max(abs(IHw.inf),abs(IHw.sup));

% -------------- random tests --------------------------------------------

nTestCases = 10;
% number of tests in which the scaling factors are increased so that the 
% "optimal" terminal region is uniformly scaled to check satisfaction of 
% C1) outside of interval(termReg)
nTestsFullyExploit = 3; 
% number of tests in which the scaling factors are shrunk to test cases in 
% which the trust region is not fully exploited
nTestsShiftPartialExploit = 4; 

resC1 = false(nTestCases,1);
resC2 = false(nTestCases,1);
tol = 1e-6;

for ii = 1:nTestCases
    % generate random terminal region
    termReg = zonotope.generateRandom('Dimension',solver.nx, ...
        'Center',solver.xEq,'NrGenerators',solver.nx*solver.termRegOrder);
    tr = interval(termReg) + solver.tr_set;

    % compute coefficients of abstraction error approximation 
    [quadCoeff,bilinCoeff,trCoeff] = ...
        solver.approxAbsErrQuad_coefficients(dynamics.sys,termReg,param.W);

    % sample random generator scaling factors and compute "optimal" 
    % terminal region
    s = rand(solver.nx*solver.termRegOrder,1); 
    termRegOpt = zonotope(solver.xEq,termReg.G*diag(s));
    % adapt generator scaling factors
    if ii <= nTestsFullyExploit + nTestsShiftPartialExploit
        s_uni = scaleZonoIntoZono(termRegOpt,tr);
        s = s*s_uni;
        if nTestsFullyExploit < ii && ii <= nTestsFullyExploit + nTestsShiftPartialExploit
            s = s*rand(1);
        end
    end
    % update "optimal" terminal region
    termRegOpt = zonotope(solver.xEq,termReg.G*diag(s));

    % evaluate abstraction error approximation
    var_dx = solver.cvx_intervalTermReg(s,termReg+zonotope(-opts.xEq));
    var_dz_quad = [var_dx;du;dw].^2;
    aux_scaleOut = solver.computeTrustRegionExploitation(s,termReg);
    absErr_eval = solver.approxAbsErrQuad_eval(var_dz_quad,aux_scaleOut, ...
        quadCoeff,bilinCoeff,trCoeff);

    % check condition [1, Def. 3, C1)]
    % ... compute coefficients of the abstraction error approximation for 
    % the "optimal" terminal region
    [quadCoeffNext,bilinCoeffNext,trCoeffNext] = ...
        solver.approxAbsErrQuad_coefficients(dynamics.sys,termRegOpt,param.W);
    var_dxNext = solver.cvx_intervalTermReg( ...
        ones(solver.nx*solver.termRegOrder,1),termRegOpt+zonotope(-solver.xEq));
    var_dzNext_quad = [var_dxNext;du;dw].^2;
    absErr_eval_opt = solver.approxAbsErrQuad_eval(var_dzNext_quad,0, ...
        quadCoeffNext,bilinCoeffNext,trCoeffNext);

    % ... check satisfaction
    if all(absErr_eval_opt <= absErr_eval + tol)
        resC1(ii) = true;
    end

    % check condition [1, Def. 3, C2)]
    % ... evaluate "true" abstraction error at "optimal" terminal region
    absErr_true = solver.computeAbsErr(dynamics,termRegOpt);

    % ... check satisfaction
    if all(absErr_true.sup <= absErr_eval + tol) && ...
            all(absErr_true.sup <= absErr_eval_opt + tol)
        resC2(ii) = true;
    end
    
end

res = all(resC1) && all(resC2);

end
