function res = test_computeTermRegNonlinSysOpt_postprocessIterate
%TEST_COMPUTETERMREGNONLINSYSOPT_POSTPROCESSITERATE - unit test
%function of postprocessIterate
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_postprocessIterate
%
% Description:
%       This script tests the post-processing of the convex [1, Eq. (17)]/
%       polynomial [2, Eq. (19)] programming solution. To this end, we 
%       generate random pairs of terminal regions (represented as a 
%       zonotope) and generator scaling factors and compare the results
%       with the naive update of the volume of the terminal region and the
%       cost function coefficients. Moreover, the correct processing of the
%       slack variables is tested using randomly generated values.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/postprocessIterate
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
%       [2] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
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

% parameters for generating random zonotopes
nx = 3;

% instantiate solver object
nu = 2;
nw = 1;
% ... algorithm settings
opts.xEq = rand(nx,1);
opts.uEq = rand(nu,1);
opts.termRegOrder = 4;
opts.verbose = 0;
% ... parameters
param.X = zonotope.generateRandom('Center',opts.xEq,'Dimension',nx);
param.U = zonotope.generateRandom('Center',opts.uEq,'Dimension',nu);
param.W = zonotope.generateRandom('Center',zeros(nw,1),'Dimension',nw);
solver = computeTermRegNonlinSysLinApproach(param,opts);

% random testing
nTestCases = 10;
nGen = nx*opts.termRegOrder;

ub_slack = 100;     % upper bound for randomly generated slack variables
lb_slack = -1e-8;   % lower bound for randomly generated slack variables

% feasible solution
resFeas = false(nTestCases,1);
for ii = 1:nTestCases
    % generate random terminal region
    termReg = zonotope.generateRandom("Dimension",nx,"NrGenerators",nGen);
    % initialize cost function
    solver = initCostFunction(solver,termReg);

    % generate random generator scaling factors
    sCost = rand(nGen,1) + 0.6;

    % evaluate function under test (requires additional input arguments)
    slack = (0 - lb_slack)*rand(10,1) + lb_slack; % -> must be non-positive for a feasible solution
    estAbsErr = interval.generateRandom('Center',zeros(nx,1),'Dimension',nx);
    tr_exploit = rand(1,1);
    [solver,auxData] = solver.postprocessIterate(sCost,slack,tr_exploit,estAbsErr,rand(1),rand(1));

    % volume of updated terminal region
    termRegOpt = zonotope(termReg.c,termReg.G*diag(sCost));
    vol_exp = volume(termRegOpt);
    % update coefficients
    costCoeff_exp = computeCoefficients(termRegOpt,solver.costExponents);

    % check results
    % -> compare volume estimates
    % -> check update of cost coefficients
    % -> check equivalence of termRegOpt and costSetNext by checking mutual
    % containment
    % -> auxData: feasibility flag is true
    % -> auxData: trust region exploitation = tr_exploit
    % -> auxData: estimate of abstraction error equivalent to estAbsErr
    if withinTol(vol_exp,solver.costSetVolume) ...
        && all(withinTol(costCoeff_exp,solver.costCoefficients)) ...
        && contains(termRegOpt,solver.costSet,'approx:st',1e-6) ...
        && contains(solver.costSet,termRegOpt,'approx:st',1e-6) ...
        && auxData.flagIsFeasible ...
        && auxData.trustRegionExploitation == tr_exploit ...
        && contains(estAbsErr,auxData.estAbsErr) ...
        && contains(auxData.estAbsErr,estAbsErr)
        resFeas(ii) = true;
    end
end

% infeasible solutions
resInfeas = false(nTestCases,1);
for ii = 1:nTestCases
    % generate random terminal region
    termReg = zonotope.generateRandom("Dimension",nx,"NrGenerators",nGen);
    % initialize cost function
    solver = initCostFunction(solver,termReg);

    % generate random generator scaling factors
    sCost = rand(nGen,1) + 0.6;

    % evaluate function under test (requires additional input arguments)
    slack = (ub_slack - lb_slack)*rand(10,1); % -> must be positive for an infeasible solution
    estAbsErr = interval.generateRandom('Center',zeros(nx,1),'Dimension',nx);
    tr_exploit = rand(1,1);
    [solver,auxData] = solver.postprocessIterate(sCost,slack,tr_exploit,estAbsErr,rand(1),rand(1));

    % volume of updated terminal region
    termRegOpt = zonotope(termReg.c,termReg.G*diag(sCost));
    vol_exp = volume(termRegOpt);
    % update coefficients
    costCoeff_exp = computeCoefficients(termRegOpt,solver.costExponents);

    % check results
    % -> compare volume estimates
    % -> check update of cost coefficients
    % -> check equivalence of termRegOpt and costSetNext by checking mutual
    % containment
    % -> auxData: feasibility flag is true
    % -> auxData: estimate of abstraction error equivalent to estAbsErr
    if withinTol(vol_exp,solver.costSetVolume) ...
        && all(withinTol(costCoeff_exp,solver.costCoefficients)) ...
        && contains(termRegOpt,solver.costSet,'approx:st',1e-6) ...
        && contains(solver.costSet,termRegOpt,'approx:st',1e-6) ...
        && ~auxData.flagIsFeasible ...
        && auxData.trustRegionExploitation == tr_exploit ...
        && contains(estAbsErr,auxData.estAbsErr) ...
        && contains(auxData.estAbsErr,estAbsErr)
        resInfeas(ii) = true;
    end
end

res = all(resFeas) && all(resInfeas);

end

%% auxiliary functions ---------------------------------------------------

function coeff = computeCoefficients(termReg,comb)

coeff = zeros(size(comb,1),1);

for ii = 1:size(comb,1)
    coeff(ii) = abs(det(termReg.G(:,comb(ii,:))));
end

end
