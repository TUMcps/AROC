function f = longQuadrotor(x,u,w)
%LONGQUADROTOR - differential equations modelling the dynamics of a 
%       perturbed longitudinal (planar motion) quadrotor (based on [1])
%
% Syntax:
%       f = LONGQUADROTOR(x,u,w)
%
% Description:
%       Dynamics equation for the planar motion of a quadrotor as defined
%       in [1, Eq. 20]. The states are the horizontal position, the
%       vertical position, the vertical velocity, the horizontal velocity,
%       the roll angle, and the roll velocity. The inputs are the total
%       thrust (u(1)) and the desired roll angle (u(2)).
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
%       [1] I. M. Mitchell et al., "Ensuring Safety for Sampled-Data 
%           Systems: An Efficient Algorithm for Filtering Potentially 
%           Unsafe Input Signals", 2016, IEEE Conference on Decision and
%           Control, pp. 7431-7438
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

    % parameters
    g = 9.81;       % [m/s^2]
    K = 0.89/1.4;    % [1/kg]
    d0 = 70;        % [1]
    d1 = 17;        % [1]
    n0 = 55;        % [1]
    
    % dynamics
    x_dot = x(3);
    z_dot = x(4);
    vx_dot = u(1)*K*sin(x(5)) + w(1);
    vz_dot = -g + u(1)*K*cos(x(5));
    phi_dot = x(6);
    phi_dotdot = -d0*x(5) - d1*x(6) + n0*u(2);

    f = [x_dot;z_dot;vx_dot;vz_dot;phi_dot;phi_dotdot];
end
