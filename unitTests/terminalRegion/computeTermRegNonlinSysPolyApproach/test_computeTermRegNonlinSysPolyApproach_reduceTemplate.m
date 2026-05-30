function res = test_computeTermRegNonlinSysPolyApproach_reduceTemplate()
%TEST_COMPUTETERMREGNONLINSYSPOLYAPPROACH_REDUCETEMPLATE - unit test
%function of reduceTemplate.
%
% Syntax:
%       res = test_computeTermRegNonlinSysPolyApproach_reduceTemplate
%
% Description:
%       Unit test function the parameterized reduction of parameterized
%       reachable sets for polynomial programming.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/reduceTemplate
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
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 


res = false(4,1);

% initialize computeTermRegNonlinSysPolyApproach object 
nx = 2;
nu = 1;
% ... parameters
param.X = interval(-10*ones(nx,1),10*ones(nx,1));
param.U = interval(-ones(nu,1),ones(nu,1));
param.W = interval(-1,1);
% ... algorithm settings
opts.xEq = zeros(nx,1);
opts.uEq = zeros(nu,1);
opts.termRegOrder = 1;
opts.verbose = 0;

opts.solverSettings = struct();
opts.solverSettings.paramReachSetOrder = 2;

solver = computeTermRegNonlinSysPolyApproach(param,opts);

% reachable set of the abstraction
R_abs = polyZonotope([opts.xEq; opts.uEq], ...
    [2, 1, 2, 0.5, 0.3, 1.5; 0, 2, 2, 1, -1, -0.7; 0.1, -1, 0, 0.2, 0.5, 0.3], ...
    [], ...
    [ 2, 1, 0, 1, 0, 1; 2, 0, 3, 0, 1, 2; ...
    0, 1, 0 2, 0, 1; 0, 1, 0, 1, 0, 1; 1, 2, 0, 1, 0, 0; ...
    0, 0, 0, 1, 0, 1; 0, 0, 0, 1, 1, 0], ...
    [solver.id.termReg; solver.id.p; solver.id.delta_c]);
R_abs = project(R_abs,1:nx);

% reachable set of the error dynamics
R_err = polyZonotope(zeros(nx+nu,1), ...
    [diag([0.5, 0.3]); zeros(nu,nx)], [diag([0.4, 0.2]); zeros(nu,nx)], ...
    eye(2), solver.id.absErr);
R_err = project(R_err,1:nx);

% original reachable set
R_unred = exactPlus(R_abs, R_err);

% random controller parameters
candP = rand(length(solver.id.p),1);

% ---------- do not enforce recursive feasibility ------------------------
flagReduceFeasible = false;

% dummy input argument
candTermReg = restoreId(polyZonotope(param.X),solver.id.delta_c);

% function under test
[R_red, mapsParamReduce] = solver.reduceTemplate(R_abs,R_err,candTermReg,candP,flagReduceFeasible);
R_red = restoreId(R_red,solver.id.var_scaling);

% check containment R_abs + R_err \subseteq R_red at initial guess
% ... set missing parameters for initial guess
candS = ones(length(solver.id.termReg),1);
candC = zeros(nx,1);
candSAbsErr = ones(nx,1);
% ... check containment
res(1,1) = checkContainment(solver,R_unred,R_red,mapsParamReduce,candS,candP,candC,candSAbsErr);

% check containment an randomly generated "candidate solution"
candS = 0.5*rand(length(solver.id.termReg),1) + 1;
candP = rand(length(solver.id.p),1);
candC = rand(nx,1);
% ... check containment
res(2,1) = checkContainment(solver,R_unred,R_red,mapsParamReduce, candS,candP,candC,candSAbsErr);

% ---------- enforce recursive feasibility -------------------------------
flagReduceFeasible = true;

% scale candidate terminal region so that it just contains the zonotope
% enclosure of the original reachable set

% ... evalute original reachable set at initial guess
candS = ones(length(solver.id.termReg),1);
candC = zeros(nx,1);
candSAbsErr = ones(nx,1);
R_eval = evaluateReachset(solver,R_unred,candS,candP,candC,candSAbsErr);
R_eval = project(R_eval,1:nx);

% ... compute fitting candidate terminal region
Z_R_eval = zonotope(R_eval);
I_R = interval(zonotope(R_eval) - Z_R_eval.c);
candTermReg = restoreId(polyZonotope(Z_R_eval.c,diag(max(I_R.sup,abs(I_R.inf)))),solver.id.delta_c);

% reduce
[R_red, mapsParamReduce] = solver.reduceTemplate(R_abs,R_err,candTermReg,candP,flagReduceFeasible);
R_red = restoreId(R_red,solver.id.var_scaling);

% check containment R_abs + R_err \subseteq R_red at initial guess
res(3,1) = checkContainment(solver,R_unred,R_red,mapsParamReduce,candS,candP,candC,candSAbsErr);

% check containment an randomly generated "candidate solution"
candS = 0.5*rand(length(solver.id.termReg),1) + 1;
candP = rand(length(solver.id.p),1);
candC = rand(nx,1);
% ... check containment
res(4,1) = checkContainment(solver,R_unred,R_red,mapsParamReduce, candS,candP,candC,candSAbsErr);


% assign output argument
res = all(res);

end


%% Auxiliary functions ---------------------------------------------------
function res = checkContainment(solver, R_unred, R_red, mapsParamReduce, candS, candP, candC,candSAbsErr)

% evaluate original reachable set
R_unred = evaluateReachset(solver,R_unred,candS,candP,candC,candSAbsErr);
R_unred = project(R_unred,1:solver.nx);

% evalute reduced reachable set
% ... compute lengths of reducing generators
s_red = zeros(2,1);
s_red = s_red + abs(resolve(mapsParamReduce.c,[candP; candC], [solver.id.var_scalar]));
for ii= 1:length(mapsParamReduce.G)
    tmp = resolve(mapsParamReduce.G{ii},[candP; candC], [solver.id.var_scalar]);
    tmp = abs(tmp)*prod([candS;candSAbsErr].^mapsParamReduce.E(:,ii));
    s_red = s_red + tmp;
end
candSRed = s_red./mapsParamReduce.initScales;

% ... auxiliary set for scaling factors
pZ_sRed = polyZonotope( ...
    zeros(length(solver.id.red),1), diag(candSRed), ...
    [], eye(length(solver.id.red)), solver.id.red);

% ... evalute  reduced reachable set
R_red = evaluateReachset(solver,R_red,candS,candP,candC,candSAbsErr);
R_red = subs(R_red,pZ_sRed,solver.id.red);
R_red = project(R_red,1:solver.nx);

% ... check containment
res = contains(zonotope(R_red), zonotope(R_unred), 'approx:st', 1e-6);

end


function R_eval = evaluateReachset(solver,R_pZ,candS,candP,candC,candSAbsErr)

% auxiliary sets for scaling factors
pZ_s = polyZonotope( ...
    zeros(length(solver.id.termReg),1), diag(candS), ...
    [], eye(length(solver.id.termReg)), solver.id.termReg);
pZ_sAbsErr = polyZonotope( ...
    zeros(length(solver.id.absErr),1), diag(candSAbsErr), ...
    [], eye(length(solver.id.absErr)), solver.id.absErr);

% plug values into pZ-representation of reachable set
R_eval = resolve(R_pZ,candP,solver.id.p);
R_eval = resolve(R_eval,candC,solver.id.delta_c);
R_eval = subs(R_eval,pZ_s,solver.id.termReg);
R_eval = subs(R_eval,pZ_sAbsErr,solver.id.absErr);

end