function res = test_computeTermRegNonlinSysLinApproach_cvx_zonoInPolyCon
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_CVX_ZONOINPOLYCON - unit test
%function of cvx_zonoInPolyConstraint.
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_cvx_zonoInPolyCon
%
% Description:
%       To test cvx_zonoInPolyConstraint, random pairs of zonotopes and 
%       polytopes are generated.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/cvx_zonoInPolyConstraint
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

nGen = opts.termRegOrder*nx;

nTestCases = 10;

% part 1: generate random pairs of zonotopes and polytopes
resRand = false(nTestCases,1);
for ii = 1:nTestCases
    % generate random sets and generator scaling factors
    Z = zonotope.generateRandom("Dimension",nx,'NrGenerators',nGen);
    s = rand(nGen,1) + 0.5;
    P = polytope.generateRandom("Dimension",nx,"IsDegenerate",false, ...
        "IsBounded",true);
    Zscaled = zonotope(Z.c,Z.G*diag(s));

    % function under test
    cIneq = solver.cvx_zonoInPolyConstraint(Z.c,Z.G,s,P);
    
    % compare results
    if contains(P,Zscaled) == all(cIneq <= 0)
        resRand(ii) = true;
    end
end

% part 2: if the zonotope is not contained, it is uniformly shrunk until
% it is contained
resCont = false(nTestCases,1);
for ii = 1:nTestCases
    % generate random sets and scaling factors
    Z = zonotope.generateRandom("Dimension",nx,'Center',zeros(nx,1), ...
        'NrGenerators',nGen);
    s = rand(nGen,1) + 0.5;
    Zscaled = zonotope(Z.c,Z.G*diag(s));

    % the center of the zonotope has to be contained in the polytope ->
    % reject and generate a new polytope if the center is not contained
    while true
        P = polytope.generateRandom("Dimension",nx,"IsDegenerate",false, ...
            "IsBounded",true);
        if contains(P,zonotope(zeros(nx,1),1e-3*eye(nx)))
            break
        end
    end
    
    if contains(P,Zscaled)
        % check containment
        cIneq = solver.cvx_zonoInPolyConstraint(Z.c,Z.G,s,P);
        if all(cIneq <= 0)
            resCont(ii) = true;
        end
    else
        % uniformly shrink the zonotope until it is contaiend in P -> loop
        % is guaranteed to terminate since P contains 
        % zonotope(zeros(nx,1),1e-3*eye(nx)) - see lines 85-91
        while true
            Z = zonotope(Z.c,0.1*Z.G);
            Zscaled = zonotope(Z.c,Z.G*diag(s));
            if contains(P,Zscaled)
                break
            end
        end
        cIneq = solver.cvx_zonoInPolyConstraint(Z.c,Z.G,s,P);
        if all(cIneq <= 0)
            resCont(ii) = true;
        end
    end
end

res = all(resRand) && all(resCont);

end
