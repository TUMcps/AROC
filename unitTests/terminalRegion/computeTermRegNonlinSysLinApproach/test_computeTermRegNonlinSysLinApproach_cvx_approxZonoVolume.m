%% 
function res = test_computeTermRegNonlinSysLinApproach_cvx_approxZonoVolume
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_CVX_APPROXZONOVOLUME - unit test
%function of cvx_approxZonoVolume.
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_cvx_approxZonoVolume
%
% Description:
%       We generate random zonotopes and sample generator scaling factors 
%       for which we check whether the tested function satisfies 
%       [1, Lemma 5].
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/cvx_approxZonoVolume
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

% instantiate solver object
nx = 3;
nu = 2;
nw = 1;
% ... algorithm settings
opts.termRegOrder = 3;
opts.xEq = rand(nx,1);
opts.uEq = rand(nu,1);
opts.verbose = 0;
% ... parameters
param.X = zonotope.generateRandom('Center',opts.xEq,'Dimension',nx);
param.U = zonotope.generateRandom('Center',opts.uEq,'Dimension',nu);
param.W = zonotope.generateRandom('Center',zeros(nw,1),'Dimension',nw);

% number of terms in the cost function
opts.solverSettings.maxCostTerms ...
    = factorial(opts.termRegOrder*nx)/(factorial(nx)*factorial(opts.termRegOrder*nx-nx));
solver = computeTermRegNonlinSysLinApproach(param,opts);

% ----------------- randomized testing -----------------------------------

% -> check whether the function satisfies the condition in  [1, Lemma 5]
nTestCases = 10;
nGen = nx*opts.termRegOrder;
resLemma5 = false(nTestCases,1);

for ii = 1:nTestCases
    % test condition in  [1, Lemma 5]
    % ...generate random zonotope & compute parameters for evaluting the 
    % cost function
    Z1 = zonotope.generateRandom("Dimension",nx,"NrGenerators",nGen);
    solver = solver.initCostFunction(Z1);
    % ...random generator scaling factors (translate to ensure increasing
    % volume of the scaled zonotope)
    s = rand(nGen,1) + 0.7;
    % ... compute volume approximation at the initial guess (Z1) and for 
    % the scaled zonotope
    approxVol_init = solver.cvx_approxZonoVolume(ones(nGen,1));
    approxVol_scaled = solver.cvx_approxZonoVolume(s);
    % ... check condition in [1, Lemma 5]
    if approxVol_scaled >= approxVol_init
        if volume(zonotope(Z1.c,Z1.G*diag(s))) >= volume(Z1)
            resLemma5(ii) = true;
        else
            resLemma5(ii) = false;
        end
    else
        resLemma5(ii) = true;
    end

    clear Z1
end

res = all(resLemma5);

end
