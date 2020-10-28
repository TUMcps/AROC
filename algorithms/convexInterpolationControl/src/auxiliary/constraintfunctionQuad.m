function [c,ceq]=constraintfunctionQuad(y,nx,uMax,uMin)
% CONSTRAINTFUNCTIONQUAD - constraints for the optimization problem to
%                          determine the parameter of the quadratic control 
%                          law
%
% Syntax:
%       [c, ceq] = CONSTRAINTFUNCTIONQUAD(y, nx, uMax, uMin)
%
% Description:
%       This function determines whether or not the quadratic control law
%       satisfies the input constraints. The optimization problem to 
%       determine the quadratic control law is solved with the  Matlab 
%       build-in function FMINCON
%
% Input Arguments:
%
%       -y:                 Parameter vector for the optimization problem
%                           (dimension: [nu*(2*nx+1),1])
%       -nx:                Number of system states x
%       -uMax:              vector containing the upper bound of the input
%                           constraints
%       -uMin:              vector containing the lower bound of the input
%                           constraints
%
% Output Arguments:
%
%       -c:             amount of constraint violation for greater than
%                       constraints
%       -ceq:           amount of constraint violation for equality
%                       constraints
%
% See Also:
%       convexInterpolationControl, optimalControlFmincon
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


    % construct matrix A, vector b and scalar d
    A = diag(y(1:nx));
    b = y(nx+1:end-1);
    d = y(end);

    % maximum or minimum has to satisfy the constraints
    c = zeros(2,1);
    xOpt = -0.5*(A\b);

    c(1) = xOpt'*A*xOpt + b'*xOpt + d - uMax;
    c(2) = -xOpt'*A*xOpt - b'*xOpt - d + uMin;

    if isnan(c(1)) || isnan(c(2))
       c = zeros(2,1);
    end

    % no equality constraints
    ceq = [];