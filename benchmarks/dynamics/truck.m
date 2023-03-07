function f = truck(x,u,w)
% TRUCK - dynamic equations for the truck benchmark model
%
% Syntax:
%       f = TRUCK(x, u, w)
%
% Description:
%       Dynamic equation for the truck benchmark (see Eq. (9) in [1]). The 
%       benchmark represents an autonomous truck with one on-axle trailer.
%       The system states are the steering angle, the vehicle orientation,
%       the orientation of the trailer, the velocity, and the x and y 
%       position of the center of mass. Control inputs to the system are 
%       the steering wheel angle velocity and the vehicle acceleration.
%
%       <<img/truck.png>>
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
%       car, robotArm
%
% References:
%       * *[1] Althoff et al. (2020)*, CommonRoad: Vehicle Models
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

    % length of the wheel-base for truck and trailer (see Table 3 in [1])
    l_wb = 3.6;
    l_wbt = 8.1;

    % dynamic equations (see Eq. (9) in [1])
    f(1,1) = u(1) + w(1);
    f(2,1) = x(4)/l_wb * tan(x(1));
    f(3,1) = -x(4)*(sin(x(3))/l_wbt + tan(x(1))/l_wb);
    f(4,1) = u(2) + w(2);
    f(5,1) = x(4) * cos(x(2));
    f(6,1) = x(4) * sin(x(2));
