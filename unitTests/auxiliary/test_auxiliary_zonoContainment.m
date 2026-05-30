function res = test_auxiliary_zonoContainment()
%TEST_AUXILIARY_ZONOCONTAINMENT - unit test
%function of zonoContainment.
%
% Syntax:
%       res = test_auxiliary_zonoContainment()
%
% Description:
%       To test zonoContainment, random pair of zonotopes are generated and 
%       the problem in [1, Corollary 4] is solved using zonoContainemnt and
%       the corresponding function from the CORA toolbox.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       -
%
% References:
%       [1] S. Sadraddini and R. Tedrake, "Linear Encodings for Polytope 
%           Containment Problems," IEEE Conference on Decision and Control, 
%           2019, pp. 4367-4372.
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

nx = 3;
order = 3;
nTestCases = 10;

% -------- random testing ------------------------------------------------

resRand = false(nTestCases,1);
tol = 1e-6;

for ii = 1:nTestCases
    Zin = zonotope.generateRandom("Dimension",nx,"NrGenerators",ceil((order-0.5)*nx));
    Zout = zonotope.generateRandom("Dimension",nx,"NrGenerators",order*nx);

    % CORA
    resCORA = contains(Zout,Zin,'approx:st',tol);

    % function under test
    [resFUT,vars] = zonoContainment(Zin,Zout,tol);

    % compare results
    if resCORA == resFUT && xor(resFUT, vars.slack > tol)
        resRand(ii) = true;
    end
end

% -------- random testing (scale circumbody to ensure containment) -------

resRandCont = false(nTestCases,1);

for ii = 1:nTestCases
    Zin = zonotope.generateRandom("Dimension",nx,"Center",zeros(nx,1), ...
        "NrGenerators",ceil((order-0.5)*nx));
    Zout = zonotope.generateRandom("Dimension",nx,"Center",zeros(nx,1), ...
        "NrGenerators",order*nx);
    % scale Zin into Zout to ensure containment
    scale = scaleZonoIntoZono(Zin,Zout);
    Zin = zonotope(Zin.c,Zin.G*scale);

    % CORA
    resCORA = contains(Zout,Zin,'approx:st',tol);

    % function under test
    [resFUT,vars] = zonoContainment(Zin,Zout,tol);

    % compare results
    if (resCORA && resFUT) && vars.slack <= tol
        resRandCont(ii) = true;
    end
end


res = all(resRand) && all(resRandCont);

end
