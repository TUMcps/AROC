function f = quadrotor2D(x,u,w)
% QUADROTOR2D - dynamic equations for the the 2D quadrotor benchmark
%
% Syntax:
%       f = QUADROTOR2D(x,u,w)
%
% Description:
%       Dynamic equation for the 2D quadrotor benchmark (see Eq. (3) in 
%       [1]). The system states are the quadrotors x- and z-position, the
%       quadrtoror orientiation, and the corresponding velocities. The
%       control inputs to the system are the thrusts geneterated by the 
%       rotors.
%
%       <<img/quadrotor2D.png>>
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
%       cart, robotArm
%
% References:
%       * *[1] Yuan et al. (2022)*, Safe-Control-Gym: A Unified Benchmark 
%              Suite for Safe Learning-Based Control and Reinforcement 
%              Learning in Robotics
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
    
    % parameter
    m = 0.027;
    Iyy = 1.4e-5;
    l = 0.0397;
    g = 9.8;
    
    % dynamic function
    f(1,1) = x(4);
    f(2,1) = x(5);
    f(3,1) = x(6);
    f(4,1) = sin(x(3))*(u(1)+u(2))/m + w(1);
    f(5,1) = cos(x(3))*(u(1)+u(2))/m - g + w(2);
    f(6,1) = (u(2)-u(1))*l/(sqrt(2) * Iyy) + w(3);
