function f = platoon(x,u,w)
% PLATOON - dynamic equations for the car benchmark model
%
% Syntax:
%       f = PLATOON(x, u, w)
%
% Description:
%
%       The benchmarks represents a platoon with 4 vehicles, where the
%       states are the relative positions and velocities betwenn the
%       vehicles, and the system inputs are the vehicle accelerations (see
%       Sec. IV in [1]).
%
%       <<img/platoon.png>>
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
%       * *[1] Schuermann et al. (2017)*, Optimal Control of Sets of 
%              Solutions to Formally Guarantee Constraints of Disturbed 
%              Linear Systems
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
% Authors:      Niklas Kochdumper, Ivan Hernandez
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    f(1,1) = x(2);
    f(2,1) = u(1) + w(1);
    f(3,1) = x(4);
    f(4,1) = u(1) - u(2) + w(1) - w(2);
    f(5,1) = x(6);
    f(6,1) = u(2) - u(3) + w(2) - w(3);
    f(7,1) = x(8);
    f(8,1) = u(3) - u(4) + w(3) - w(4);

end