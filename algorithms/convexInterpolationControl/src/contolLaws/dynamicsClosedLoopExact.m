function f = dynamicsClosedLoopExact(x,u,p,nx,dynamics)
% DYNAMICSCLOSEDLOOPEXACT - system dynamics for the closed-loop system
%                           controlled with the exact control law
%
% Syntax:
%       f = DYNAMICSCLOSEDLOOPEXACT(t,x,u,p,nx,dynamics)
%
% Description:
%       This function implements the closed loop system dynamics for the 
%       case that the system is controlled by the nonlinear exact control 
%       law. Thereby, the original system dynamics of the benchmark models 
%       are extended by auxiliary states that store the linear part of the
%       exact control law and that keep the control input constant during
%       one time step. Further, the control input computed with the exact
%       control law and insterted in the open-loop system dynamic of the
%       benchmark modlel, which yields the equations for the closed-loop
%       system. 
%
% Input Arguments:
%
%       -x:         system states x (dimension: [nx,1])
%       -u:         disturbances = input to the closed loop controlled 
%                   system (dimension: [nx,1])
%       -p:         parameter vector. Contains the optimal control inputs
%                   for all vertices of the parallelotope 
%                   (dimension: [nu*(2^nx),1])
%       -nx:        number of system states
%       -dynamics:  function handle to the function implementing the
%                   open-loop dynamics of the system
%
% Output Arguments:
%
%       -f:     value of the dynamic funcion = time derivative of x
%
% See Also:
%       convexInterpolationControl, dynamcisClosedLoopLinear
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

    % system dimensions
    nx_ = length(x)-nx;
    nu = length(p)/(2^nx_);

    % extract data needed for control law implementation
    index = 1;                                
    for i = 1:2^nx_                        
        u_(:,i) = p(index:index+nu-1);
        index = index + nu;
    end

    % compute the alpha values of the extreme points
    alpha=[1 -1];
    for i=1:nx_-1
        alpha=[ones(1,2^i) -ones(1,2^i); alpha alpha];
    end

    % Control law
    x_ = x(nx+1:end);

    for i = 1:2^nx_
        lambda(i,1) = (1-x_(1))^(-0.5*(alpha(1,i)-1)) * x_(1)^(0.5*(alpha(1,i)+1));
    end

    for i = 1:2^nx_
        for j = 2:nx_
            lambda(i,1) = lambda(i,1) * (1-x_(j))^(-0.5*(alpha(j,i)-1)) * x_(j)^(0.5*(alpha(j,i)+1));
        end
    end

    U = u_ * lambda;

    % extended system dynamic 
    f(1:nx,1) = dynamics(x(1:nx),U,u);

    f(nx+1:length(x),1) = zeros(length(x)-nx,1);
