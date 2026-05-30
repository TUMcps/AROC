function f = chainMSD_10(x,u,w)
%CHAINMSD_10 - differential equations modeling the system consisting of a
%       chain of 10 (coupled) mass-spring-damper systems (based on the cart 
%       system).
%
% Syntax:
%       f = CHAINMSD_10(x,u,w)
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
%       cart
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
% Copyright (c) 2021 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % parameters
    m = 1;      % mass
    k = 0.8;    % gain of the stiffnes term
    d = 0.33;   % gain of the damping term

    % dynamics
    f(1,1) = x(2);
    f(2,1) = -d/m*x(2)^2 - k/m*x(1)^3 ...
        + d/m*(x(4)-x(2))^2 + k/m*(x(3)-x(1))^3 ...
        + u(1)/m + w(1);
    idx = 2; ii = 2;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);   
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + d/m*(x(idx+4)-x(idx+2))^2 + k/m*(x(idx+3)-x(idx+1))^3 ...
        + u(ii)/m + w(ii);      
    idx = idx+2; ii = ii+1;
    f(idx+1,1) = x(idx+2);
    f(idx+2,1) = -d/m*(x(idx+2)-x(idx))^2 - k/m*(x(idx+1)-x(idx-1))^3 ...
        + u(ii)/m + w(ii);   
end
