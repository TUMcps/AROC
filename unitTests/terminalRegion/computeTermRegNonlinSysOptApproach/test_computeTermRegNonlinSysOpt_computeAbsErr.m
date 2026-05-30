function res = test_computeTermRegNonlinSysOpt_computeAbsErr
%TEST_COMPUTETERMREGNONLINSYSOPT_COMPUTEABSERR - unit test function
%of computeAbsErr.
%
% Syntax:
%       res = test_computeTermRegNonlinSysOpt_computeAbsErr
%
% Description:
%       To test the correctness of the implementation, we randomly sample
%       initial states, control inputs, and disturbances and check whether
%       the nonlinear part of the dynamics is contained in the computed set
%       of abstraction errors.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysOpt/computeAbsErr
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

% ------------- init parameters and dynamical system ---------------------

% dynamical system
benchmark = 'cart';

% sets of admissible states, control inputs, and disturbances
width = [6; 6];
param.X = interval(-width, width);
width = 14;
param.U = zonotope(interval(-width,width));
width = [0.1;0.1];
param.W = interval(-width,width);

% algorithm settings 
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.termRegOrder = 5;
opts.timeStep = 0.1;
opts.verbose = 0;

% settings for reachability analysis and computation of derivatives
% dummy settings required for computing derivatives 
opts.reachOpts.tensorOrder = 2;
opts.reachOpts.zonotopeOrder = 100;
opts.reachOpts.reductionTechnique = 'girard';
opts.reachOpts.errorOrder = 1;
opts.reachOpts.intermediateOrder = 1;

% testing-related parameters
nTestCases = 20;

% ----------- random testing - linearization-based approach --------------
% -> discrete-time system, tensor order 2
opts.sysType = termRegSysType.DT;
opts.nonlinSysTermRegAlg = 'lin';
opts.integrator = discretizeDynamics.RK2;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);


% generate random terminal region
termReg = zonotope.generateRandom('Dimension',solver.nx,'Center',solver.xEq, ...
    'NrGenerators',solver.nx*solver.termRegOrder);
% scale terminal region into set of admissible states
s_termReg = scaleZonoIntoZono(termReg,zonotope(param.X));
termReg = zonotope(termReg.c,s_termReg*termReg.G);

% compute set of abstraction errors
absErr = computeAbsErr(solver,dynamics,termReg);

% sample initial state, control input, and disturbance
x0 = randPoint(termReg,nTestCases,'extreme');
u = randPoint(solver.Z_U,nTestCases,'extreme');
w = randPoint(solver.W,nTestCases,'extreme');

% linearize dynamical system
f_exp = dynamics.sys.mFile([solver.xEq;solver.uEq],zeros(solver.nw,1));
[jac_xu,jac_w] = dynamics.sys.jacobian([solver.xEq;solver.uEq],zeros(solver.nw,1));
AB_d = jac_xu;
Ed = jac_w;

res_lin2_DT = false(nTestCases,1);
for ii = 1:nTestCases
    xSim = dynamics.sys.mFile([x0(:,ii);u(:,ii)],w(:,ii));
    xSimLin =  f_exp + [AB_d,Ed]*[x0(:,ii)-solver.xEq;u(:,ii)-solver.uEq;w(:,ii)];
    df = xSim-xSimLin;
    res_lin2_DT(ii) = contains(absErr,df(1:solver.nx));
end

% ----------- random testing - linearization-based approach --------------
% -> sampled-data-time system, tensor order 2
opts.sysType = termRegSysType.SD;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% instantiate dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% generate random terminal region
termReg = zonotope.generateRandom('Dimension',solver.nx,'Center',solver.xEq, ...
    'NrGenerators',solver.nx*solver.termRegOrder);
% scale terminal region into set of admissible states
s_termReg = scaleZonoIntoZono(termReg,zonotope(param.X));
termReg = zonotope(termReg.c,s_termReg*termReg.G);

% compute set of abstraction errors
absErr = computeAbsErr(solver,dynamics,termReg);

% sample initial state, control input, and disturbance
x0 = randPoint(termReg,nTestCases,'extreme');
u = randPoint(solver.Z_U,nTestCases,'extreme');
w = randPoint(solver.W,nTestCases,'extreme');

% linearize dynamical system
f_exp = dynamics.sys.mFile([solver.xEq;solver.uEq],zeros(solver.nw,1));
[jac_xu,jac_w] = dynamics.sys.jacobian([solver.xEq;solver.uEq],zeros(solver.nw,1));
AB_c = jac_xu;
Ec = jac_w;

res_lin2_SD = false(nTestCases,1);
for ii = 1:nTestCases
    xDot = dynamics.sys.mFile([x0(:,ii);u(:,ii)],w(:,ii));
    xDotLin =  f_exp + [AB_c,Ec]*[x0(:,ii)-solver.xEq;u(:,ii)-solver.uEq;w(:,ii)];
    df = xDot-xDotLin;
    res_lin2_SD(ii) = contains(absErr,df(1:solver.nx));
end

% assign output argument
res = all(res_lin2_DT) && all(res_lin2_SD);

end
