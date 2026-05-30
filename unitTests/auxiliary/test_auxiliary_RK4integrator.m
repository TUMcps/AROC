function res = test_auxiliary_RK4integrator
%TEST_AUXILIARY_RK4INTEGRATOR - unit test of the function RK4integrator.
%
% Syntax:
%       res = test_auxiliary_RK4integrator
%
% Description:
%       This function tests the function RK4integrator, which is designed
%       for time-discretizing nonlinear dynamical systems using CasADi's 
%       integrator function [1] (note that the CasADi function cannot be 
%       passed to CORA for reachability analysis).
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% test requires casadi
import casadi.*

try
    nTestCases = 20;

    % initialize dynamical system and parameters
    dynamics = @(x,u,w) robotArm(x,u,w);
    % ... set of admissible states
    width = [pi/2; pi/2; pi; pi];
    X = interval(-width, width);
    % ... set of admissible control inputs
    width = [3;1];
    U = interval(-width,width);
    % ... set of uncertain disturbances
    width = 10*[0.01;0.01;0.01;0.01];
    W = interval(-width,width);
    % sampling time
    dt = 0.1;

    % instantiate casadi integrator object
    
    % ... define auxiliary variables
    xx = sym('x',[X.dim,1],'real');
    uu = sym('u',[U.dim+W.dim,1],'real');
    x_k = MX.sym('x_k',X.dim,1); % statet at time step k
    u_k = MX.sym('u_k',U.dim+W.dim,1); % input at time step k
    % ... integrator
    symDynamics = @(x,u) dynamics(x,u(1:U.dim),u(U.dim+(1:W.dim)));
    symDynamics = symDynamics(xx,uu);
    symDynamics = matlabFunction(symDynamics,'Vars',{xx,uu});
    casInt = integrator('Dx','rk',struct('x',x_k,'p',u_k, ...
        'ode',symDynamics(x_k,u_k)),0,dt);

    % sample random initial states, control inputs, and disturbances
    x0 = randPoint(X,nTestCases);
    u = randPoint(U,nTestCases);
    w = randPoint(W,nTestCases);
    
    % simulate and compare results
    res = false(nTestCases,1);
    for ii = 1:nTestCases
        % ... expected result
        x_exp = full(getfield(casInt('x0',x0(:,ii),'p',[u(:,ii);w(:,ii)]),'xf'));
        % ... function under test
        x_ut = RK4integrator(x0(:,ii),u(:,ii),w(:,ii),dynamics,dt);
        % ... compare results
        res(ii) = all(withinTol(x_ut,x_exp,1e-6));
    end

    res = all(res);
catch
    res = false;
    
end

end
