function f = cartpole(x,u,w)
%CARTPOLE - differential equations modeling the dynamics of a cartpole
%
% Syntax:
%       f = CARTPOLE(x,u,w)
%
% Description:
%       Dynamics equation for the cartpole benchmark model. The system
%       states are the position of the cart, the angle of the pole, the 
%       velocity of the car, and the angular velocity of the pole. The
%       control input is the force acting on the cart.
%
%       <<img/cartpole.png>>
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
%       -
%
% References:
%       [1] J. Theis, "Sum-of-Squares Applications in Nonlinear Controller 
%           Synthesis", UC Berkeley, 2012
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
% Copyright (c) 2021 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% parameters
l = 1;   % [m]
m1 = 1;   % [kg]
m2 = 0.1;  % [kg]
g = 9.81;   % [m/s^2]

% dynamics, see [1, Eq. (71)]
x_dot = x(3);
psi_dot = x(4);
v_dot = (m2*g*cos(x(2))*sin(x(2))-m2*l*x(4)^2*sin(x(2)) + u(1) + w(1))/ ...
    (m1+m2-m2*cos(x(2))^2);
psi_dotdot = ((m1+m2)*g*sin(x(2))-m2*l*x(4)^2*sin(x(2))*cos(x(2)) + ...
    cos(x(2))*(u(1)+w(1)))/(m1*l+m2*l-m2*l*cos(x(2))^2) + w(2);

f = [x_dot;psi_dot;v_dot;psi_dotdot];

end
