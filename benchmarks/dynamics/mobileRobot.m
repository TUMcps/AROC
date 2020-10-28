function f = mobileRobot(x,u,w)
% MOBILEROBOT - dynamic equations for the mobile robot benchmark model
%
% Syntax:
%       f = MOBILEROBOT(x, u, w)
%
% Description:
%       Dynamic equation for the mobile robot benchmark model (see Eq. (6),
%       (7),(8),(14) and (15) in [1]). The benchmarks describes a Pioneer 
%       3DX mobile robotThe system states are x and y position of the 
%       center, the robot orientation, and the angular velocities of the 
%       two actuated wheels. Control inputs are the torques that act on the 
%       two actuated wheels.
%
%       <<img/mobileRobot.png>>
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
%       car, platoon
%
% References:
%       * *[1] Ivanjo et al. (2010)*, Modelling of mobile robot dynamics.
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

    % system parameter
    m = 28.05;
    r = 0.095;
    b = 0.32;
    d = 0.0578;
    I_0 = 9.24e-6;
    I_A = 175e-1;
    K = 35e-7;
    
    A = 0.25*m*r^2 + ((I_A + m*d^2)*r^2)/b^2 + I_0;
    B = 0.25*m*r^2 - ((I_A + m*d^2)*r^2)/b^2;
    
    % differential equation
    f(1,1) = r/2*(x(4)+x(5))*cos(x(3));
    f(2,1) = r/2*(x(4)+x(5))*sin(x(3));
    f(3,1) = r/b*(x(4)-x(5));
    f(4,1) = 1/(A^2 - B^2)*(A*u(1) - A*K*x(4) - B*u(2) + B*K*x(5)) + w(1);
    f(5,1) = 1/(A^2 - B^2)*(-B*u(1) + B*K*x(4) + A*u(2) - A*K*x(5)) + w(2); 