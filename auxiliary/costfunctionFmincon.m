function cost = costfunctionFmincon(y,nx,N,xf,Q,R)
% COSTFUNCTIONFMINCON - objective function for the optimal control problems
%                       if solved with FMINCON
%
% Syntax:
%       [c, ceq] = COSTFUNCTIONFMINCON(y, nx, N, xf, Q, R)
%
% Description:
%       This function implements the objective function for the optimal 
%       control problem, for the case that the optimal control problems
%       are solved with Matlab build-in function FMINCON
%
% Input Arguments:
%
%       -y:                 Parameter vector for the optimization problem
%                           (y = [x,u])
%       -nx:                Number of system states x
%       -N:                 Number of time steps for the optimal control
%                           problem
%       -xf:                desired final state at the end of the optimal
%                           control problem
%       -Q:                 weighting matrix for the system states
%                           (dimension: [nx,nx])
%       -R:                 weighting matris for the control inputs
%                           (dimension: [nu,nu])
%
% Output Arguments:
%
%       -cost:          value of the objective function
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


    % obtaining states and inputs
    x=y(1:nx*N);
    u=y(nx*N+1:end);

    % cost function based on sum of used inputs and difference of final state
    % to desired final state
    cost=u'*R*u + (x-xf)'*Q*(x-xf);
