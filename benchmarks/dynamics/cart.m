function f = cart(x,u,w)
% CART - dynamic equations for cart benchmark model
%
% Syntax:
%       f = CART(x, u, w)
%
% Description:
%       Dynamic equation for the cart benchmark model. The benchmark 
%       represents a cart that is coupled to the environment by a spring 
%       and a damper, where both the stiffness of the spring and the 
%       damping of the damper are nonlinear functions of the cart position. 
%       The system states are the position and the velocity of the cart. 
%       The input to the system is an external force acting on the cart.
%
%       <<img/cart.png>>
%
% Input Arguments:
%
%       -x:     system states x (dimension: [nx,1])
%       -u:     control inputs u (dimension: [nu,1]) 
%       -w:     disturbances w (dimension: [nw,1])
%
% Output Arguments:
%
%       -f:     value of the dynamic function = time derivative of x
%
% See Also:
%       car, doubleIntegrator, robotArm
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

    % Parameter
    m = 1;              % mass
    k = 1;              % gain of the stiffnes term
    d = 1;              % gain of the damping term
    
    % Dynamic equations
    f(1,1) = x(2) + w(1);
    f(2,1) = 1/m * (-d * x(2)^2 - k * x(1)^3 + u) + w(2);