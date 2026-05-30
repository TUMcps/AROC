function res = test_computeTermRegNonlinSysLinApproach_cvx_intervalTermReg
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_CVX_INTERVALTERMREG - unit test
%function of cvx_intervalTermReg.
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_cvx_intervalTermReg
%
% Description:
%       We generate random pairs of zonotopes and generator scaling
%       factors and check correctness of the resulting interval hull using 
%       CORA as well as containment of the scaled terminal region.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/cvx_intervalTermReg
% 
% References:
%       -
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

solver = computeTermRegNonlinSysLinApproach(param,opts);

nTestCases = 10;

res = false(nTestCases,1);
for ii = 1:nTestCases
    % generate random zonotope and scaling factors
    termReg = zonotope.generateRandom("Dimension",nx,"NrGenerators",opts.termRegOrder*nx);
    s = rand(nx*opts.termRegOrder,1) + 0.6;
    termReg_sc = zonotope(termReg.c,termReg.G*diag(s));

    % evaluate function under test
    IH_semiEdgeLength = solver.cvx_intervalTermReg(s,termReg);
    IH = interval(zonotope(termReg.c,diag(IH_semiEdgeLength)));

    % compute interval enclosure using CORA
    IH_exp = interval(termReg_sc);
    IH_semiEdgeLength_exp = 0.5*(IH_exp.sup-IH_exp.inf);

    % check results
    res(ii) = contains(IH,termReg_sc) ...
        && all(withinTol(IH_semiEdgeLength_exp,IH_semiEdgeLength));
end

res = all(res);

end
