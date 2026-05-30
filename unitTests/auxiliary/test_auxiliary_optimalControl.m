function res = test_auxiliary_optimalControl()
%TEST_AUXILIARY_OPTIMALCONTROL - unit test function for the optimalControl
%class.
%
% Syntax:
%       res = TEST_AUXILIARY_OPTIMALCONTROL()
%
% Description:
%       Solves an optimal control problem (similar to the center trajectory
%       computation) and checks satisfaction of the input constraints as 
%       well as similarity with the simulated solution.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @optimalControl
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
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% handle to continous-timedynamics function
sys = @(x,u,w) cart(x,u,w);

% get benchmark parameters
Param = param_cart();

% define algorithm options
Opts = [];
Opts.N = 20;        
Opts.dt = Param.tFinal/Opts.N;

% function handles for the cost function 
R = eye(Param.U.dim);
Q = eye(Param.X.dim);
stageCost = cell(Opts.N,1);
stageCost(:) =  {@(x,u) Opts.dt*u'*R*u};
terminalCost = @(x) (x-Param.xf)'*Q*(x-Param.xf);

% function under test ----------------------------------------------------

% optimal control
% ... instantiate optimalControl object
OCP = optimalControl(sys,stageCost,terminalCost,Param.X,Param.U,Param.W,Opts.N,Opts.dt);
% ... compute an initial guess by linearly interpolating between
% the initial state x0 and the goal state xf
x0 = center(Param.R0);
y0 = OCP.initialGuessInterpolate(x0,Param.xf);
% solve optimal control problem
sol = OCP.solveOCP(y0,x0);
pred = [];
pred.u = sol.u;
pred.x = sol.x;

ocp_results = results([],[],pred);

% check results -------------------------------------------------------------

% check input constraints
res = ocp_results.checkSimInputs(Param.U);

% check error between predicted and simulated final state
xSim = zeros(Param.X.dim,Opts.N+1); wNom = zeros(Param.W.dim,1);
for kk = 1:Opts.N
    [~,x_temp] = ode45(@(t,x)sys(x,pred.u(:,kk),wNom), ...
        [0 Opts.dt],xSim(:,kk)');
    xSim(:,kk+1) = x_temp(end,:)';
end
width = 1e-6*ones(Param.X.dim);
res = res && ocp_results.checkFinalSimPointsInSet( ...
    interval(-width,width)+xSim(:,end));

end