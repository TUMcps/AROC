function u = exactControlLaw(x,zonoR,uCorner)
% EXACTCONTROLLAW - implements the nonlinear control law
%
% Syntax:
%       u = EXACTCONTROLLAW(x,zonoR,uCorner)
%
% Description:
%       This function implements the nonlinear control law which is based
%       on exact convex combinations (see Sec. 4 in [1]).
%
% Input Arguments:
%
%       -x:         system states x (dimension: [nx,1])
%       -zonoR:     parallelotope for which the control law should be
%                   calculated
%       -uCorner:   matrix storing the optimal control inputs for the
%                   parallelotope vertices
%                   (dimension: [nu,2^nx])
%
% Output Arguments:
%
%       -u:             resuling control input = value of the control law
%                       (dimension: [nu,1])
%
% See Also:
%       convexInterpolationControl, dynamicsClosedLoopExact
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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

    % extract parallelotope parameter
    Z = zonoR.Z;
    G = Z(:,2:end);
    c = Z(:,1);
    
    nx = length(c);

    % compute x_ (see Eq. (14) in [1])
    x_ = 0.5*ones(size(x)) + 0.5*(G\(x-c));

    % compute the alpha values of the extreme points
    alpha = [1 -1];
    for i = 1:nx-1
        alpha = [ones(1,2^i) -ones(1,2^i); alpha alpha];
    end

    % Control law
    for i = 1:2^nx
        lambda(i,1) = (1-x_(1))^(-0.5*(alpha(1,i)-1)) * x_(1)^(0.5*(alpha(1,i)+1));
    end

    for i = 1:2^nx
        for j = 2:nx
            lambda(i,1) = lambda(i,1) * (1-x_(j))^(-0.5*(alpha(j,i)-1)) * x_(j)^(0.5*(alpha(j,i)+1));
        end
    end

    % convex combination
    u = uCorner * lambda;