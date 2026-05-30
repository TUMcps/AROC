function res = test_computeTermRegNonlinSysLinApproach_cvx_zonoContSoftCon
%TEST_COMPUTETERMREGNONLINSYSLINAPPROACH_CVX_ZONOCONTSOFTCON - unit test
%function of cvx_zonoContSoftConstraint.
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_cvx_zonoContSoftCon
%
% Description:
%       To test cvx_zonoContSoftConstraint (see also [1, Remark 4], random
%       pair of zonotopes are generated and the problem in [2, Corollary 4]
%       is solved to obtain the auxiliary variables. The solution is passed
%       to cvx_zonoContSoftConstraint to check that the equality and
%       inequality constraints are (not) satisfied.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/cvx_zonoContSoftConstraint
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
%       [2] S. Sadraddini and R. Tedrake, "Linear Encodings for Polytope 
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

nTestCases = 5;
tol = 1e-6;
nGen = nx*solver.termRegOrder;

% -------- random testing ------------------------------------------------

resRand = false(nTestCases,1);
for ii = 1:nTestCases
    Zin = zonotope.generateRandom("Dimension",nx,"NrGenerators",ceil((opts.termRegOrder-0.5)*nx));
    Zout = zonotope.generateRandom("Dimension",nx,"NrGenerators",nGen);

    % solve [2, Corollary 4] to obtain auxiliary variables
    [~,varsST] = zonoContainment(Zin,Zout,tol);

    % evaluate constraint functions
    [cEq,cIneq] = solver.cvx_zonoContSoftConstraint( ...
        Zin.c,Zin.G,Zout.c,Zout.G,varsST.Gamma,varsST.gamma, ...
        abs(varsST.Gamma),abs(varsST.gamma),1,varsST.slack);

    if all(all(abs(cEq) <= tol)) && all(all(cIneq <= tol))
        resRand(ii) = true;
    end

    % check if nominal inequality constraints (slack=0) are violated, if
    % Zin is not contained in Zout
    if ~contains(Zout,Zin,'approx:st',tol) 
        [cEq,cIneq] = solver.cvx_zonoContSoftConstraint( ...
            Zin.c,Zin.G,Zout.c,Zout.G,varsST.Gamma,varsST.gamma, ...
            abs(varsST.Gamma),abs(varsST.gamma),1,0);
        if ~(all(all(abs(cEq) <= tol)) && any(any(cIneq >= tol)))
            resRand(ii) = false;
        end
    end
end

% -------- random testing (scale circumbody to ensure containment) -------

resRandCont = false(nTestCases,1);
for ii = 1:nTestCases
    Zin = zonotope.generateRandom("Dimension",nx,"Center",zeros(nx,1), ...
        "NrGenerators",ceil((opts.termRegOrder-0.5)*nx));
    Zout = zonotope.generateRandom("Dimension",nx,"Center",zeros(nx,1), ...
        "NrGenerators",nGen);
    % scale Zin into Zout to ensure containment
    scale = scaleZonoIntoZono(Zin,Zout);
    Zin = zonotope(Zin.c,Zin.G*scale);

    % solve [2, Corollary 4] to obtain auxiliary variables
    [~,varsST] = zonoContainment(Zin,Zout,tol);

    % evaluate constraint functions
    [cEq,cIneq] = solver.cvx_zonoContSoftConstraint( ...
        Zin.c,Zin.G,Zout.c,Zout.G,varsST.Gamma,varsST.gamma, ...
        abs(varsST.Gamma),abs(varsST.gamma),1,varsST.slack);

    if all(all(abs(cEq) <= tol)) && all(all(cIneq <= tol))
        resRandCont(ii) = true;
    end
end


res = all(resRand) && all(resRandCont);

end
