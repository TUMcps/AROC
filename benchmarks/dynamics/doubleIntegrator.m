function f = doubleIntegrator(x,u,w)
% DOUBLEINTEGRATOR - dynamic equations for the double-integrator benchmark 
%                    model
%
% Syntax:
%       f = DOUBLEINTEGRATOR(x, u, w)
%
% Description:
%       Dynamic equation for the double-integrator benchmark model. The 
%       benchmark represents a rigid body that glides on a plane without
%       friction. The system states are the position and the velocity of 
%       the rigid body. The input to the system is an external force that 
%       acts on the rigid body.
%
%       <<img/doubleIntegrator.png>>
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
%       car, cart, robotArm
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

    f(1,1) = x(2);
    f(2,1) = u(1) + w(1);