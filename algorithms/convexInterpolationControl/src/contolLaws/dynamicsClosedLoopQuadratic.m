function f = dynamicsClosedLoopQuadratic(x,u,p,nx,dynamics)
% DYNAMICSCLOSEDLOOPQUADRATIC - system dynamics for the closed-loop system
%                               controlled with the quadratic control law
%
% Syntax:
%       f = DYNAMICSCLOSEDLOOPQUADRATIC(t,x,u,p,nx,dynamics)
%
% Description:
%       This function implements the closed loop system dynamics for the 
%       case that the system is controlled by the quadratic control 
%       law. Thereby, the original system dynamics of the benchmark models 
%       are extended by auxiliary states that store the states at the 
%       beginning of the time step. The auxiliary states guarantee that the 
%       control inputs are kept constant during one time step. Further, the
%       control inputs are computed with the quadratic 
%       control law and insterted into the open-loop system dynamic of the
%       benchmark modlel, which yields the equations for the closed-loop
%       system. 
%
% Input Arguments:
%
%       -x:         system states x (dimension: [nx,1])
%       -u:         disturbances = input to the closed loop controlled 
%                   system (dimension: [nx,1])
%       -p:         parameter vector. Contains the parameter of the optimal
%                   control law (dimension: [nu*(2*nx+1),1])
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
    nu = length(p) / (2*nx_ + 1);

    % extract data needed for control law implementation
    A = cell(nu,1);
    b = cell(nu,1);
    o = cell(nu,1);

    index = 1; 
    for i = 1:nu
        A{i} = diag(p(index:index+nx_-1));
        index = index + nx_;
        b{i} = p(index:index+nx_-1);
        index = index + nx_;
        o{i} = p(index);
        index = index + 1;
    end


    % Control law
    for i = 1:nu
       U(i) = x(nx+1:end)'*A{i}*x(nx+1:end) + b{i}'*x(nx+1:end) + o{i}; 
    end


    % extended system dynamic 
    f(1:nx,1) = dynamics(x(1:nx),U,u);

    f(nx+1:length(x),1) = zeros(length(x)-nx,1);