function f = controlledJetEngine(x,u,w)
%CONTROLLEDJETENGINE - differential equations modeling the dynamics of the 
%       Moore-Greitzer model of a jet engine
%
% Syntax:
%       f = CONTROLLEDJETENGINE(x,u,w)
%
% Description:
%       Dynamics equation for the controlled jet engine model. 
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
%       [1] M. Krstic et al., "Nonlinear and Adaptive Control Design", 
%           John Wiley and Sons, 1995.
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % dynamics
    x1_dot = -x(2) - 1.5*x(1)^2 - 0.5*x(1)^3 + w(1);
    x2_dot = u(1);

    f = [x1_dot;x2_dot];
end
