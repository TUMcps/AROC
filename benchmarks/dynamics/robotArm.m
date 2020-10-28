function f = robotArm(x,u,w)
% ROBOTARM - dynamic equations for the robot arm benchmark model
%
% Syntax:
%       f = ROBOTARM(x, u, w)
%
% Description:
%       Dynamic equations for the robot arm benchmark model. The benchmark
%       represents a planar three degree-of-freedom robot with two joints.
%       The system states are the two joint angles and the two joint
%       angular velocities. The input to the system are the two joint
%       torques.
%
%       <<img/robotArm.png>>
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
%       car, cart, doubleIntegrator
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
    m1 = 1;             % mass body 1
    m2 = 1;             % mass body 2
    I1 = 1;             % inertia body 1
    I2 = 1;             % inertia body 2
    l1 = 0.2;           % lenght body 1
    l2 = 0.2;           % lenght body 2
    r1 = 0.1;           % position COM body 1
    r2 = 0.1;           % position COM body 2

    a = I1 + I2 + m1*r1^2+m2*(l1^2+r2^2);
    b = m2*l1*r2;
    c = I2+m2*r2^2;
    
    % Dynamic matrices
    det = c*(a+2*b*cos(x(2)))-(c+b*cos(x(2)))^2;
    M_inv = 1/det * [c -c-b*cos(x(2));-c-b*cos(x(2)) a+2*b*cos(x(2))];
    D = [-b*sin(x(2))*x(4) -b*sin(x(2))*(x(3)+x(4));b * sin(x(2))*x(3) 0];
    
    % Dynamic equation   
    f(1,1) = x(3) + w(1);
    f(2,1) = x(4) + w(2);
    
    f(3:4,1) = M_inv * (-D * [x(3);x(4)] + [u(1);u(2)]) + [w(3);w(4)];   