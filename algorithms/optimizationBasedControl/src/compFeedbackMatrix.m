function K = compFeedbackMatrix(x,A,B)
% COMPFEEDBACKMATRIX - compute the feedback matrix for the control law
%
% Syntax:
%       c = CHECKCONSTRAINTS(R,K,u_ref,Opts)
%
% Description:
%       This function computes the feedback matrix for the control law u =
%       u_ref + K(x-x_ref) from the current value of the optimization
%       variables. The optimization variables are the weights of the state
%       and input weigthing matrix, and the feedback matrix is computed
%       with the LQR approach.
%
% Input Arguments:
%
%       -x:     current value for the optimization variables
%       -A:     dynamic matrix of the linear of linearized system
%       -B:     input matrix of the linear of linearized system
%
% Output Arguments:
%
%       -K:     computed feedback matrix for the control law 
%               u = u_ref + K(x-x_ref)
%
% See Also:
%       optimizationBasedControl, conFun, costFun
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
% Authors:      Ivan Hernandez, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % extract state and input weighting matrices
    % Q = diag(1,x(1),...,x(nx-1)), R = diag(x(n),...,x(nx+nu-1)
    nx = size(A,1);

    Q = diag([1;x(1:nx-1)]);
    R = diag(x(nx:end));

    % compute the feedback matrix with the LQR approach
    K = lqr(A,B,abs(Q),abs(R));
    K = -K;