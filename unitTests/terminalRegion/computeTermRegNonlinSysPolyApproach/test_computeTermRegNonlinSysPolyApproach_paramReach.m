function res = test_computeTermRegNonlinSysPolyApproach_paramReach()
%TEST_COMPUTETERMREGNONLINSYSPOLYAPPROACH_PARAMREACH - unit test 
% function of paramReach
%
% Syntax:
%       res = test_computeTermRegNonlinSysPolyApproach_paramReach
%
% Description:
%       Unit test function of the parameterized reachability analysis for
%       computing robust control invariant sets using polynomial controller
%       synthesis. For nonlinear sampled-data systems, the parameterized
%       approximations of the time-point reachable sets proposed in [1,
%       Eq. (17)] are subsets of the over-approximative reachable sets
%       computed using the algorithms implemented in CORA (given the same
%       parameters and settings are chosen). Therefore, we randomly sample 
%       initial sets and controllers and compute the parameterized 
%       reachable set as well as an over-approximation using CORA and check 
%       containment.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/paramReach
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

benchmark = 'cart';

% sets of admissible states, control inputs, and disturbances
% -> shrunken sets compared to parameters from manual to avoid reachable
% set explosion
width = [2;2];
param.X = interval(-width, width);
widthU = 3;
param.U = interval(-widthU,widthU);
width = [0.1;0.1];
param.W = interval(-width,width);

% settings
opts.timeStep = 0.01; % time step must be very small to avoid reachable set explosion
opts.termRegOrder = 5;
opts.xEq = zeros(param.X.dim,1);
opts.uEq = zeros(param.U.dim,1);
opts.sysType = termRegSysType.SD;
opts.nonlinSysTermRegAlg = 'poly';
opts.verbose = 0;

% instantiate solver object
% ... set parmeterized reachset order sufficiently high to avoid order
% reduction
opts.solverSettings.paramReachSetOrder = 1000;
solver = computeTermRegNonlinSysPolyApproach(param,opts);

% initialize dynamics
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% LQR (stabilizing control law to avoid reachabl set explosion)
AB = dynamics.sys.jacobian([solver.xEq;solver.uEq],zeros(solver.nw,1));
[minusKd,~,~] = lqrd(AB(1:solver.nx,1:solver.nx),AB(1:solver.nx,solver.nx+(1:solver.nu)), ...
    eye(solver.nx),eye(solver.nu),opts.timeStep);
Kd = -minusKd;

% settings for over-approximative reachability analysis 
reachOpts = struct();
reachOpts.alg = 'poly';
reachOpts.tensorOrder = 3;
reachOpts.timeStep = opts.timeStep;
% ... linearize around equilibrium
reachOpts.linearizationPoint = [solver.xEq;solver.uEq];
% ... choose orders as in @computeTermRegNonlinSysPolyApproach/paramReach
reachOpts.zonotopeOrder = 100;
reachOpts.taylorTerms = 6;
nGen = solver.nx*(opts.termRegOrder+1);
order = ceil(nGen/(solver.nx+solver.nu));
reachOpts.zonotopeOrder = 50;       
reachOpts.intermediateOrder = 50;
reachOpts.errorOrder = 20;
% % -> errorOrder: reduce time-point reachable set before quadMap
% reachOpts.errorOrder = order; 
% % -> intermediateOrder: reduce quadMap of time-point reachable set
% reachOpts.intermediateOrder = ...
%         ceil(0.5*nGen*(nGen+1)/(solver.nx+solver.nu));

% ------- random testing -------------------------------------------------

nTestCases = 10;
resCORA = false(nTestCases,1);

for ii = 1:nTestCases
    % generate random initial set
    termReg = zonotope.generateRandom("Dimension",solver.nx,"Center",solver.xEq,...
        "NrGenerators",solver.nx*solver.termRegOrder);
    % if necessary, scale the terminal region into X
    if ~contains(param.X,termReg)
        scale_termReg = scaleZonoIntoZono(termReg,zonotope(param.X));
        termReg = zonotope(termReg.c,termReg.G*scale_termReg);
    end
    % polyZonotope representation
    termReg = polyZonotope(termReg.c,termReg.G,[],eye(solver.nx*solver.termRegOrder),solver.id.termReg);
    termReg_sc = restoreId(termReg,solver.id.delta_c);

    % control law
    Zctrl = Kd*termReg;
    % polyZonotope representation
    ctrlSet = polyZonotope(solver.uEq,Zctrl.G,[],eye(solver.nx*solver.termRegOrder),solver.id.termReg);
    % recover controller parameters
    ctrlParam = [ctrlSet.c; ctrlSet.G(:)]./widthU;

    % parameterized reachability analysis
    [Rparam,~] = solver.paramReach(dynamics,termReg_sc,solver.pZu_vp,ctrlParam,1,false);
    Rparam = resolve(Rparam,ctrlParam,solver.id.p);
    Rparam = resolve(Rparam,zeros(solver.nx,1),solver.id.delta_c);

    % compute over-approximate reachable set using CORA
    % ... parameters for reachability analysis
    tmp_params = dynamics.reachParams;
    tmp_params.R0 = stack(termReg, ctrlSet);

    % ... reachability analysis
    [Rtp,~] = reach(dynamics.sys,tmp_params,reachOpts);
    Rcora = project(Rtp.timePoint.set{end},1:solver.nx);

    % check containment: 
    % ... the parameterized approximation is expected to be
    % contained in the results computed using CORA (more precisely, the
    % respective zonotope enclosures)
    if contains(zonotope(Rcora),zonotope(Rparam),'approx:st',1e-6)
        resCORA(ii) = true;
    end
end

res = all(resCORA);

end
