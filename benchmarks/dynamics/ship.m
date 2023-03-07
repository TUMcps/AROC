function f = ship(x,u,w)
% SHIP - dynamic equations for the ship benchmark model
%
% Syntax:
%       f = SHIP(x,u,w)
%
% Description:
%       Dynamic equation for the ship benchmark model (see Sec. 6 in [1]). 
%       The benchmark represents a 3 degree of freedom model of a container
%       vessel. The system states are the x and y position, the
%       orientation, the velocities in x and y in the vessel coordinate
%       frame, and the angular velocity. Control inputs to the system are 
%       the forces and the moment acting on the ship.
%
%       <<img/ship.png>>
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
%       * *[1] Krasowski et al. (2022)*, CommonOcean: Vessel Models
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % parameter (see Table 2 in [1])
    m = 24.56e6;                 % total mass [kg]
    x_g = -2.55;                 % distance to center of gravity [m]
    Iz = 54.99e9;                % inertial around z-axis [kg m^2]
    Xu = -150.64e3;              % surge aligned force coefficient
    Yv = -218.6;                 % sway lateral force coefficient
    Yr = 9.277e3;                % yaw lateral force coefficient
    Nv = -14.23e3;               % sway moment coefficient
    Nr = -1.113e6;               % yaw moment coefficient
    Xu_ = -1.8e6;                % surge-rate lateral force coefficient
    Yv_ = -26.54e6;              % sway-rate lateral force coefficient
    Yr_ = -283.4e6;              % yaw-rate lateral force coefficient
    Nr_ = -44.85e9;              % yaw-rate moment coefficient

    % dynamic model (see Section 6 in [1])
    Cm = Iz*m - Nr_*m - Yv_*Iz + Nr_*Yv_ - m^2*x_g^2 + 2*m*x_g*Yr_ - Yr_^2;

    Minv = [1/(m-Xu_) 0 0; ...
            0 (Iz-Nr_)/Cm (Yr_-m*x_g)/Cm; ...
            0 (Yr_-m*x_g)/Cm (m-Yv_)/Cm];

    C_RB = [0 0 -m*(x_g*x(6) + x(5));
            0 0 m*x(6);
            m*(x_g*x(6) + x(5)) -m*x(6) 0];

    C_A = [0 0 Yv_*x(5) + Yr_*x(6);
           0 0 -Xu_*x(4);
           -Yv_*x(5)-Yr_*x(6) Xu_*x(4) 0];

    C = C_RB + C_A;

    D = [-Xu 0 0; 
         0 -Yv -Yr; 
         0 -Nv -Nr];

    f(1,1) = cos(x(3))*x(4) - sin(x(3))*x(5);
    f(2,1) = sin(x(3))*x(4) + cos(x(3))*x(5);
    f(3,1) = x(6);
    f(4:6,1) = Minv*(-C*x(4:6) - D*x(4:6) + [u(1);u(2);u(3)] + w(1:3)); 
end