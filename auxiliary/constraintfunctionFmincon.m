function [c,ceq]=constraintfunctionFmincon(y,nx,nu,N,dt,x0,dynamics)
% CONSTRAINTFUNCTIONFMINCON - constraints for the optimal control problems
%                             if solved with FMINCON
%
% Syntax:
%       [c,ceq] = CONSTRAINTFUNCTIONFMINCON(y, nx, nu, N, dt, x0, dynamics)
%
% Description:
%       This function determines whether or not the constraints for the
%       optimal control problems are satisfied, for the case that the
%       optimal control problmes are solved with Matlab build-in function
%       FMINCON
%
% Input Arguments:
%
%       -y:         parameter vector for the optimization problem 
%                   (y = [x,u])                        
%       -nx:        number of system states x
%       -nu:        number of control inputs u
%       -N:         number of time steps for the optimal control problem
%       -dt:        time step size for the optimal control problem
%       -x0:        initial state at the beginning of the optimal control
%                   problem
%       -dynamics:  function handle pointing to the function that
%                   implements the system dynamics
%
% Output Arguments:
%
%       -c:             amount of constraint violation for greater than
%                       constraints
%       -ceq:           amount of constraint violation for equality
%                       constraints
%
% See Also:
%       convexInterpolationControl, optimalControlFmincon
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


    % only nonlinear constraint is to check if the state trajectory is
    % continuous. Must be checked since we use a multiple-shooting algorithm
    x=y(1:nx*N);
    u=y(nx*N+1:end);
    x=reshape(x,[nx,N]);
    u=reshape(u,[nu,N]);
    x=[x0,x];
    xconstr=zeros(nx,N);

    % compute states based on inputs and numerical integration
    for k=1:N
        [~,x_temp]=ode45(@(t,x)dynamics(t,x,u(:,k)),[0 dt],x(:,k)');
        xconstr(:,k) = x_temp(end,:)';
    end

    % check difference between assumed and actual states of the state trajectory
    ceq=y(1:nx*N)-reshape(xconstr,[N*nx,1]); 
    c=[];