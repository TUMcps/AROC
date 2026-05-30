function f = platoon4(x,u,w)
% PLATOON4 - dynamic equations for the platoon benchmark
%
% Syntax:
%       f = PLATOON4(x, u, w)
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
%       -
%
% References:
%       [1] Schuermann et al., "Optimal Control of Sets of Solutions to 
%           Formally Guarantee Constraints of Disturbed Linear Systems",
%           2017, American Control Conference
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

    x1_dot = x(2);
    x2_dot = u(1) + w(1);
    x3_dot = x(4);
    x4_dot = u(1) - u(2) + w(1) - w(2);
    x5_dot = x(6);
    x6_dot = u(2) - u(3) + w(2) - w(3);
    x7_dot = x(8);
    x8_dot = u(3) - u(4) + w(3) - w(4);

    f = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot];
end
