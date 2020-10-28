function f = dynamicsClosedLoopLinear(x,u,nx,nu,dynamics)
% DYNAMICSCLOSEDLOOPLINEAR - system dynamics for the closed-loop system
%                            controlled with the linear control law
%
% Syntax:
%       f = DYNAMICSCLOSEDLOOPLINEAR(t,x,u,nx,nu,dynamics)
%
% Description:
%       This function implements the closed loop system dynamics for the 
%       case that the system is controlled by the linear control law.
%       Thereby, the original system dynamics of the benchmark models 
%       are extended by auxiliary states that store the input zonotope as 
%       well as the states at the beginning of the time step. The 
%       implementation with auxiliary states guarantees that the control 
%       inputs are kept constant during one time step. Further, the value
%       from the input zonotope (=control input) is insterted into the 
%       open-loop system dynamic of the benchmark modlel, which yields the
%       equations for the closed-loop system. 
%
% Input Arguments:
%
%       -x:         system states x (dimension: [nx,1])
%       -u:         disturbances = input to the closed loop controlled 
%                   systme (dimension: [nx,1])
%       -nx:        number of system states
%       -nu:        number of control inputs to the system
%       -dynamics:  function handle to the function implementing the
%                   open-loop dynamics of the system
%
% Output Arguments:
%
%       -f:     value of the dynamic funcion = time derivative of x
%
% See Also:
%       convexInterpolationControl, dynamcisClosedLoopExact
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


    % extended system dynamics (state vector + input zonotope + auxiliary states)
    f(1:nx,1) = dynamics(x(1:nx),x(nx+1:nx+nu),u);
    
    f(nx+1:length(x),1) = zeros(length(x)-nx,1);