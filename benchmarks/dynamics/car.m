function f = car(x,u,w)
% CAR - dynamic equations for the car benchmark model
%
% Syntax:
%       f = CAR(x, u, w)
%
% Description:
%       Dynamics equation for the car benchmark model (see Eq. (19) in [1]). 
%       The benchmark represents the kinematik single track model of an 
%       autonomous vehicle. The system states are the vehicle orientation,
%       the velocity, and the x and y position of the center of mass. 
%       Control inputs to the system are the steering wheel angle and the 
%       vehicle acceleration.
%
%       <<img/car.png>>
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
%       [1] Schuermann et al., "Convex interpolation control with formal 
%           guarantees for disturbed and constrained nonlinear systems",
%           2017, International Conference on Hybrid Systems: Computation
%           and Control
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

    v_dot = u(1) + w(1);
    psi_dot = u(2) + w(2);
    x_dot = cos(x(2))*x(1);
    y_dot = sin(x(2))*x(1);

    f = [v_dot;psi_dot;x_dot;y_dot];
end
