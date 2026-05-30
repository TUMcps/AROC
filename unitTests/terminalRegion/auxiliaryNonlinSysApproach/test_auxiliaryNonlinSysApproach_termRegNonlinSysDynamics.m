function res = test_auxiliaryNonlinSysApproach_termRegNonlinSysDynamics
%TEST_AUXILIARYNONLINSYSAPPROACH_TERMREGNONLINSYSDYNAMICS - unit test
%function of termRegNonlinSysDynamics (constructor) and methods defined in 
% the class file.
%
% Syntax:
%       res = test_auxiliaryNonlinSysApproach_termRegNonlinSysDynamics()
% 
% Description:
%       This function tests the correct instantiation of objects of the
%       class termRegNonlinSysDynamics and methods defeind in the class
%       file.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @termRegNonlinSysDynamics
%
% References:
%       [1] M. Althoff et al., "Reachable set computation for uncertain 
%           time-varying linear systems", 2011, International Conference on 
%           Hybrid Systems: Computation and Control.
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

% dynamical system
benchmark = 'longQuadrotor';

str = ['dynamicsHandle = @(x,u,w)',benchmark,'(x,u,w);'];
eval(str);

% set of admissible states
lb = [-1.7; 0.3; -0.8; -1; -pi/12; -pi/2]; 
ub = [1.7; 2.0; 0.8; 1.0; pi/12; pi/2];
param.X = interval(lb, ub);
% set of admissible control inputs
g = 9.81; K = 0.89/1.4;
lb = [-1.5+g/K; -pi/12]; ub = [1.5+g/K; pi/12];
param.U = interval(lb, ub);
% set of uncertain disturbances
width = [0.1; 0.01]; 
param.W = interval(-width, width);

% ... algorithm settings
opts.nonlinSysTermRegAlg = 'lin';
opts.timeStep = 0.05;
g = 9.81; K = 0.89/1.4;
opts.xEq = [0;1;0;0;0;0];
opts.uEq = [g/K;0];
opts.termRegOrder = 5;
opts.verbose = 0;

%% ---------------- test: discrete-time system ----------------------------
opts.sysType = termRegSysType.DT;

% ---------------- test constructor --------------------------------------
opts.integrator = discretizeDynamics.RK4;

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% discretize and linearize dynamical system
integrator_FH = opts.integrator.getFunHandle;
dynamicsDT = @(x,u,w) integrator_FH(x,u,w,dynamicsHandle,opts.timeStep);
x = sym('x',[solver.nx,1]); 
u = sym('u',[solver.nu,1]);
w = sym('w',[solver.nw,1]);
dynamicsDT = dynamicsDT(x,u,w);
A = matlabFunction(jacobian(dynamicsDT,x),'Vars',{x,u,w});
B = matlabFunction(jacobian(dynamicsDT,u),'Vars',{x,u,w});
E = matlabFunction(jacobian(dynamicsDT, w),'Vars',{x,u,w});
Alin = A(solver.xEq,solver.uEq,zeros(solver.nw,1));
Blin = B(solver.xEq,solver.uEq,zeros(solver.nw,1));
AB_lin = [Alin, Blin];
Elin = E(solver.xEq,solver.uEq,zeros(solver.nw,1));

% execute function under test
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% checks:
% ... correct nonlinear system type instantiated
% ... correct dimension of (augmented) state-space
% ... correct dimension of uncertain input space
% ... correct uncertain input set for nonlinear reachability analysis
% (disturbance set)
% ... correct sampling time set
% ... correct state and input matrices of the linearized dynamics
% ... correct reachable set due to the set of disturbances
res_DT_RK4 = isa(dynamics.sys,'nonlinearSysDT') && ...
    dynamics.sys.nrOfDims == param.X.dim+param.U.dim && ...
    dynamics.sys.nrOfInputs == param.W.dim && ...
    (contains(dynamics.reachParams.U,param.W) && ...
    contains(param.W,dynamics.reachParams.U)) && ...
    (dynamics.dt == opts.timeStep && ...
    dynamics.reachParams.tFinal == opts.timeStep) && ...
    all(withinTol(AB_lin,dynamics.AB_d.C,1e-12),'all') && ...
    (contains(Elin*solver.W,dynamics.RlinW,'approx:st',1e-6) && ...
    contains(dynamics.RlinW,Elin*solver.W,'approx:st',1e-6));
% ---------------- test method computeReachSetAbsErr ---------------------

% random input set
randU = cartProd(zonotope.generateRandom("Dimension",solver.nx), ...
    zonotope(zeros(solver.nu,1)));
% execute function under test
tmp = dynamics.computeReachSetAbsErr(randU.c,randU.G);
Rerr = zonotope(tmp.c,tmp.G);

% check result:
% ... Rerr must be identical to U
randU = project(randU,1:solver.nx);
res_DT_reach = contains(Rerr,randU,'approx:st',1e-6) && ...
    contains(randU,Rerr,'approx:st',1e-6);

%% ---------------- test: sampled-data system ----------------------------
opts.sysType = termRegSysType.SD;

% ---------------- test constructor --------------------------------------

% instantiate solver object
solver = computeTermRegNonlinSysLinApproach(param,opts);

% execute function under test
dynamics = termRegNonlinSysDynamics(benchmark,opts.sysType,solver,opts,param);

% linearize and discretize dynamical system
x = sym('x',[solver.nx,1]); 
u = sym('u',[solver.nu,1]);
w = sym('w',[solver.nw,1]);
dynamicsCT = dynamicsHandle(x,u,w);
Afun = matlabFunction(jacobian(dynamicsCT,x),'Vars',{x,u,w});
Bfun = matlabFunction(jacobian(dynamicsCT,u),'Vars',{x,u,w});
Efun = matlabFunction(jacobian(dynamicsCT, w),'Vars',{x,u,w});
Ac = Afun(solver.xEq,solver.uEq,zeros(solver.nw,1));
Bc = Bfun(solver.xEq,solver.uEq,zeros(solver.nw,1));
Ec = Efun(solver.xEq,solver.uEq,zeros(solver.nw,1));
Ad = expm(Ac*opts.timeStep);
tmp = integral(@(t)(expm(Ac*(opts.timeStep-t))),0,...
    opts.timeStep,'ArrayValued',true);
Bd = tmp*Bc;
AB_d = [Ad,Bd];
Ed = tmp*Ec;

% checks - constructor
% ... correct nonlinear system type instantiated
% ... correct dimension of (augmented) state-space
% ... correct dimension of uncertain input space
% ... correct uncertain input set for nonlinear reachability analysis
% (disturbance set)
% ... correct sampling time set
% ... correct state and input matrices of the linearized dynamics
% ... reachable set due to the set of disturbances contains its
% discrete-time version
res_SD = isa(dynamics.sys,'nonlinearSys') && ...
    dynamics.sys.nrOfDims == param.X.dim+param.U.dim && ...
    dynamics.sys.nrOfInputs == param.W.dim && ...
    (contains(dynamics.reachParams.U,param.W) && ...
    contains(param.W,dynamics.reachParams.U)) && ...
    (dynamics.dt == opts.timeStep && ...
    dynamics.reachParams.tFinal == opts.timeStep) && ...
    all(withinTol(AB_d,dynamics.AB_d.C,1e-12),'all') && ...
    contains(dynamics.RlinW,Ed*solver.W,'approx:st',1e-6);

% ---------------- test method computeReachSetAbsErr ---------------------

% compute particular solution using CORA
linSys = linearSys(Ac,Bc,[],[],[],[],Ec);
delta_Ulin = zonotope(zeros(solver.nx,1),Ec*solver.W.G);
Rerr_exp = particularSolution_timeVarying(linSys,delta_Ulin,opts.timeStep,6);

% execute function under test
delta_Ulin = cartProd(delta_Ulin,zonotope(zeros(solver.nu,1)));
tmp = dynamics.computeReachSetAbsErr(delta_Ulin.c,delta_Ulin.G);
Rerr = zonotope(tmp.c,tmp.G);

% check result:
% ... Rerr must be centered at the origin
% ... Rerr must be contained in Rerr_exp (the truncation error term in [1,
% Theorem 2])
res_SD_reach = all(withinTol(Rerr.c-zeros(solver.nx,1),1e-12)) && ...
    contains(Rerr_exp,Rerr,'approx:st',1e-6);

% assign output argument
res = res_DT_RK4 && res_DT_reach && res_SD && res_SD_reach;

end
