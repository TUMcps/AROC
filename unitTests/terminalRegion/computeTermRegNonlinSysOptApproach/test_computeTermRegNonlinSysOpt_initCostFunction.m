function res = test_computeTermRegNonlinSysOpt_initCostFunction
%TEST_COMPUTETERMREGNONLINSYSOPT_INITCOSTFUNCTION - unit test 
% function of initCostFunction.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_initCostFunction
%
% Description:
%       Two cases have to be distinguished: (a) maximizing the volume of 
%       the terminal region itself or (b) maximizing the volume of a subset 
%       of the terminal region (with lower order), see also [1, Remark 2]. 
%       In both cases, we check whether the correct flag is assigned as 
%       well as whether summing up the cost function coefficients yields 
%       the volume of the cost set. 
%       Moreover, we check whether ...
%       ... case (a): the cost set is identical to the initial guess for 
%           the terminal region (tested by checking for mutual containment).
%       ... case (b): the cost set is contained in the initial guess for 
%           the terminal region.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/initCostFunction
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
 
% paramters for instantiating the computeTermRegNonlinSysLinApproach object
% based on the cart benchmark

% sets of admissible states, control inputs, and disturbances
width = [6; 6];
X = interval(-width, width);
width = 14;
U = zonotope(interval(-width,width));
width = [0.1;0.1];
W = interval(-width,width);

% algorithm settings
xEq = zeros(X.dim,1);
uEq = zeros(U.dim,1);
termRegOrder = 5;

nx = X.dim;

nTestCases = 10;

% -------------- test case: no reduction required ------------------------

solverSettings.maxCostTerms ...
    = factorial(termRegOrder*nx)/(factorial(nx)*factorial(termRegOrder*nx-nx));

% instantiate computeTermRegNonlinSysLinApproach object
solver = computeTermRegNonlinSysOpt(X,U,W,xEq,uEq,termRegOrder,0,solverSettings);
assert(solver.flagCostExact, ['Error in computeTermRegNonlinSysOpt ' ...
    'constructor - property flagCostExact must be true!'])

resNoRed = false(nTestCases,1);
for ii = 1:nTestCases
    % random zonotope as initial guess for terminal region
    initialGuess = zonotope.generateRandom("Dimension",nx, ...
        "NrGenerators",termRegOrder*nx,'Center',xEq);
    
    % evaluate function under test
    solver = solver.initCostFunction(initialGuess);
    
    % compare results:
    % ... solver.costSet identical to termReg -> mutual containment 
    % ... volume of solver.costSet = (2^nx)*sum(solver.costCoefficients)
    resNoRed(ii) = ...
        contains(initialGuess,solver.costSet,'approx:st',1e-6') ...
        && contains(solver.costSet,initialGuess,'approx:st',1e-6') ...
        && withinTol(volume(initialGuess),2^nx*sum(solver.costCoefficients),1e-10);
end

% -------------- test case: reduction required ---------------------------

solverSettings.maxCostTerms ...
    = floor(0.5*factorial(termRegOrder*nx)/(factorial(nx)*factorial(termRegOrder*nx-nx)));

% instantiate computeTermRegNonlinSysOpt object
solver = computeTermRegNonlinSysOpt(X,U,W,xEq,uEq,termRegOrder,0,solverSettings);
assert(~solver.flagCostExact, ['Error in computeTermRegNonlinSysOpt ' ...
    'constructor - property flagCostExact must be false!'])

resRed = false(nTestCases,1);
for ii = 1:nTestCases
    % random zonotope as initial guess for terminal region
    initialGuess = zonotope.generateRandom("Dimension",nx, ...
        "NrGenerators",termRegOrder*nx,'Center',xEq);
    
    % evaluate function under test
    solver = solver.initCostFunction(initialGuess);
    
    % compare results:
    % ... solver.costSet contained in termReg, but not vice versa 
    % ... volume of solver.costSet = (2^nx)*sum(solver.costCoefficients)
    resRed(ii) = ...
        contains(initialGuess,solver.costSet,'approx:st',1e-6') ...
        && withinTol(volume(solver.costSet),2^nx*sum(solver.costCoefficients),1e-10);
end

res = all(resNoRed) && all(resRed);

end
