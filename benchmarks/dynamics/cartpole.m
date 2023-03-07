function f = cartpole(x,u,w)
%CARTPOLE - differential equations modeling the dynamics of a cartpole
%
% Syntax:
%       f = CARTPOLE(x,u,w)
%
% Description:
%       Dynamic equation for the cartpole benchmark model. The system
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
%       [1] S. Geva "A Cartpole Experiment Benchmark for Trainable 
%	    Controllers", 1993
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
m_c = 1;   % [kg]
m_p = 0.1;  % [kg]
g = 9.81;   % [m/s^2]

% dynamics
f(1,1) = x(3);
f(2,1) = x(4);
f(3,1) = (m_p*l*x(4)^2*sin(x(2))*cos(x(2))^2 - (m_c + m_p)*g*cos(x(2))*sin(x(2)) + cos(x(2))^2*u)/(4/3*(m_c + m_p)^2*l - m_p*(m_c + m_p)*l*cos(x(2))^2) + (u(1) + m_p*l*x(4)^2*sin(x(2)))/(m_c + m_p) + w(1);
f(4,1) = ((m_c + m_p)*g*sin(x(2)) - m_p*l*x(4)^2*sin(x(2))*cos(x(2)) - cos(x(2))*u(1))/(4/3*(m_c + m_p)*l - m_p*l*cos(x(2))^2) + w(2);

end
