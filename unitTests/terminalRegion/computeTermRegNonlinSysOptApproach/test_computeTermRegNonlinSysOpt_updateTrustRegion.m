function res = test_computeTermRegNonlinSysOpt_updateTrustRegion
%TEST_COMPUTETERMREGNONLINSYSOPT_UPDATETRUSTREGION - unit test
%function of updateTrustRegion.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_updateTrustRegion
%
% Description:
%       This function tests the correctness of the implemented trust region
%       update strategy - to this end, we generate random terminal regions,
%       generator scaling factors, and sets of abstraction error to check
%       the three cases for updating the trust region, see [1, App. II].
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/updateTrustRegion
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

% parameters for random sampling
nx = 3;

% instantiate solver object
nu = 2;
nw = 1;
% ... algorithm settings
opts.xEq = rand(nx,1);
opts.uEq = rand(nu,1);
opts.termRegOrder = 5;
opts.verbose = 0;
% ... parameters
param.X = zonotope.generateRandom('Center',opts.xEq,'Dimension',nx);
param.U = zonotope.generateRandom('Center',opts.uEq,'Dimension',nu);
param.W = zonotope.generateRandom('Center',zeros(nw,1),'Dimension',nw);

solver = computeTermRegNonlinSysOpt(param.X,param.U,param.W,...
    opts.xEq,opts.uEq,opts.termRegOrder,opts.verbose);

nGen = nx*opts.termRegOrder;
nTestCases = 20;
res_c1 = false(nTestCases,1);
res_c2 = false(nTestCases,1);
res_c3 = false(nTestCases,1);
for ii = 1:nTestCases

    % set trust region
    solver.tr_set = zonotope(interval.generateRandom( ...
        'Dimension',nx,'Center',zeros(nx,1)));

    % random cost set
    costSet = zonotope.generateRandom('Dimension',nx,'NrGenerators',nGen,...
        'Center',zeros(nx,1));

    % case 1: termReg_opt is contained in interval(termReg)
    % -> trust region exploitation = 0
    tr_exploit = 0;
    auxDataIter = termRegNonlinSysSolutionAuxData(true,costSet,tr_exploit, ...
        interval.generateRandom('Center',zeros(solver.nx,1)),rand(1),rand(1));
    absErr_true = interval.generateRandom('Center',zeros(solver.nx,1));
    % ... function under test
    [solver_next,tr_action] = solver.updateTrustRegion(auxDataIter,absErr_true);
    % ... check results
    if strcmp(tr_action,'no action') ...
            && contains(solver_next.tr_set,solver.tr_set) ...
            && contains(solver.tr_set,solver_next.tr_set)
        res_c1(ii) = true;
    end

    % case 2: terminal region intersects with boundary of 
    % interval(solPrev.termReg) + opts.scvx.tr_set
    % -> trust region exploitation = 1
    tr_exploit = 1;
    auxDataIter = termRegNonlinSysSolutionAuxData(true,costSet,tr_exploit, ...
        interval.generateRandom('Center',zeros(solver.nx,1)),rand(1),rand(1));
    absErr_true = interval.generateRandom('Center',zeros(solver.nx,1));
    % ... function under test
    [solver_next,tr_action] = solver.updateTrustRegion(auxDataIter,absErr_true);
    % ... check results
    if strcmp(tr_action,'enlarge') ...
            && contains(solver_next.tr_set,solver.tr_set) ...
            && ~contains(solver.tr_set,solver_next.tr_set)
        res_c2(ii) = true;
    end

    % case 3: otherwise
    % -> 0 < trust region exploitation < 0.99
    % -> generate random sets of abstraction errors
    tr_exploit = 0.89*rand + 0.1;
    absErr_iter = interval.generateRandom('Dimension',nx,'Center',zeros(nx,1));
    auxDataIter = termRegNonlinSysSolutionAuxData( ...
        true,costSet,tr_exploit,absErr_iter,rand(1),rand(1));
    absErr_true = 0.98*interval.generateRandom('Dimension',nx,'Center',zeros(nx,1));
    % ... function under test
    [solver_next,tr_action] = solver.updateTrustRegion(auxDataIter,absErr_true);
    % ... check results
    if any((auxDataIter.estAbsErr.sup-absErr_true.sup)./absErr_true.sup > 0) ...
           && strcmp(tr_action,'shrink') ...
            && ~contains(solver_next.tr_set,solver.tr_set) ...
            && contains(solver.tr_set,solver_next.tr_set)
       res_c3(ii) = true;
    elseif ~all((auxDataIter.estAbsErr.sup-absErr_true.sup)./absErr_true.sup > 0) ...
           && strcmp(tr_action,'no action') ...
            && contains(solver_next.tr_set,solver.tr_set) ...
            && contains(solver.tr_set,solver_next.tr_set)
       res_c3(ii) = true;
    end
end

res = all(res_c1) && all(res_c2) && all(res_c3);

end
