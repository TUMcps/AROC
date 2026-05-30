function res = test_computeTermRegNonlinSysPolyApproach_approxAbsErrPWLin
%TEST_TERMINALREGION_APPROXABSERRPWLIN - unit test of the piece-wise linear
% approximation of the abstraction error.
%
% Syntax:
%       res = test_computeTermRegNonlinSysPolyApproach_approxAbsErrPWLin
%
% Description:
%       This script tests whether the piece-wise linear approximation of 
%       the abstraction error proposed in [1, Example 1] satisifes the 
%       condition in [1, Eq. (20)]. To this end, we generate random 
%       pairs of terminal regions (represented as a zonotope) and generator 
%       scaling factors, etc. for the cart benchmark.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_coefficients, 
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_con
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_eval
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
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
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
opts.nonlinSysTermRegAlg = 'poly';
opts.verbose = 0;

% instantiate solver object
solver = computeTermRegNonlinSysPolyApproach(param,opts);

% initialize dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% set trust region
solver.tr_set = zonotope(zeros(solver.nx,1),eye(solver.nx));

% indices of optimization variables
idxVars = solver.computeVariableIndices(solver.id,solver.id.red, ...
    1,solver.termRegOrder*solver.nx,1,1,1,1);

% -------------- random tests --------------------------------------------

nTestCases = 10;
% number of tests in which the scaling factors are increased to test cases in 
% which the trust region is fully exploited
nTestsFullyExploit = 3; 
% number of tests in which the scaling factors are shrunk to test cases in 
% which the trust region is not fully exploited
nTestsShiftPartialExploit = 4; 

resCon = false(nTestCases,1);
res20a = false(nTestCases,1);
res20b = false(nTestCases,1);

tol = 1e-6;

for ii = 1:nTestCases
    % generate random terminal region
    termReg = zonotope.generateRandom('Dimension',solver.nx, ...
        'Center',solver.xEq,'NrGenerators',solver.nx*solver.termRegOrder);
    % pZ representation with parameterized center
    pZ_c = polyZonotope(zeros(solver.nx,1),eye(solver.nx),[], ...
        eye(solver.nx),solver.id.delta_c);
    termReg_pZ = polyZonotope(termReg.c,termReg.G,[], ...
        eye(solver.nx*solver.termRegOrder),solver.id.termReg);
    termReg_pZ = exactPlus(termReg_pZ,pZ_c);

    IH0 = interval(zonotope(termReg.c,termReg.G) - solver.xEq);
    dx0 = max(abs(infimum(IH0)),abs(supremum(IH0)));
    tr = interval(-dx0,dx0) + solver.tr_set;
    dx_tr = interval(tr).sup;

    % compute coefficients and constraints of abstraction error approximation 
    coefficients = ...
        solver.approxAbsErrPwLin_coefficients(dynamics,termReg);
    [cIneqStruct, cIneqStructAux] = solver.approxAbsErrPwLin_con( ...
        termReg,coefficients,idxVars);

    % sample random generator scaling factors and center shift
    s = rand(solver.nx*solver.termRegOrder,1); 
    delta_c = 0.2*rand(solver.nx,1);
    termRegOpt = zonotope(solver.xEq+delta_c,termReg.G*diag(s));
    % adapt generator scaling factors
    if ii <= nTestsFullyExploit + nTestsShiftPartialExploit
        s_uni = scaleZonoIntoZono(termRegOpt,tr);
        s = s*s_uni;
        if nTestsFullyExploit < ii && ii <= nTestsFullyExploit + nTestsShiftPartialExploit
            s = s*rand(1);
        end
    end
    % set "optimal" terminal region
    termRegOpt = zonotope(solver.xEq+delta_c,termReg.G*diag(s));

    % compute optimzation variables to evaluate the constraints
    y = zeros(idxVars.nVars,1);
    y(idxVars.s) = s;
    y(idxVars.delta_c) = delta_c;
    % ... auxiliary variables: box enclosure of termRegOpt
    % -> positive direction
    tmp_pos = (termRegOpt.c(solver.argAbsErr) ...
        + sum(abs(termRegOpt.G(solver.argAbsErr,:)),2));
    % -> negative direction
    tmp_neg = (-termRegOpt.c(solver.argAbsErr) ...
        + sum(abs(termRegOpt.G(solver.argAbsErr,:)),2));
    dx_opt = max([tmp_pos,tmp_neg],[],2);
    y(idxVars.auxSAbsErr(1)) = max(dx_opt./dx0);
    % .... auxiliary variables: exploitation of trust region
    y(idxVars.auxSAbsErr(2)) = max((dx_opt-dx0)./(dx_tr-dx0));
    % ... auxiliary variables: abstraction error scaling factors
    absErr_eval = solver.approxAbsErrPwLin_eval(termReg_pZ,s,delta_c,coefficients);
    y(idxVars.sAbsErr) = absErr_eval./coefficients.termReg;

    % check constraints
    if all(cIneqStruct.Aineq*y <= cIneqStruct.bineq+tol) && ...
            all(cIneqStructAux.Aineq*y <= cIneqStructAux.bineq+tol) && ...
            length(cIneqStruct.bineq) == cIneqStruct.nCon && ...
            length(cIneqStructAux.bineq) == cIneqStructAux.nCon
        resCon(ii) = true;
    end

    % check the condition in [1, Eq. (20a)] 
    % ... compute approximation at termRegOpt 
    coefficients_opt = ...
        solver.approxAbsErrPwLin_coefficients(dynamics,termRegOpt);
    if all(coefficients_opt.termReg <= absErr_eval+tol)
        res20a(ii) = true;
    end
    
    % check the condition in [1, Eq. (20b)]
    % ... evauate abstraction error at termRegOpt
    absErr_opt = solver.computeAbsErr(dynamics,termRegOpt);
    if all(absErr_opt.sup <= absErr_eval+tol)
        res20b(ii) = true;
    end    
end

res = all(resCon) && all(res20a) && all(res20b);

end
