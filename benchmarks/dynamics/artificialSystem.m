function f = artificialSystem(x,u,w)
% ARTIFICIALSYSTEM - dynamic equations for the artificial system benchmark 
%                    model
%
% Syntax:
%       f = ARTIFICIALSYSTEM(x, u, w)
%
% Description:
%       Dynamic equation for the artifical system benchmark model as 
%       defined in Eq. (19) in [1]. 
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
%       * *[1] Yu et al. (2013)*, Tube MPC scheme based on robust control 
%              invariant set with application to Lipschitz nonlinear 
%              systems, Systems & Control Letters
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % system dynamics
    f(1,1) = -x(1) + 2*x(2) + 0.5*u(1);
    f(2,1) = -3*x(1) + 4*x(2) -0.25*x(2)^3 - 2*u(1) + w(1);
    
    % convert from minutes to seconds
    f = f./60;
end