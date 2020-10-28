function [res,L_,J_] = checkConstraint(x,u,L,J,Opts)
% CHECKCONSTRAINT - check if the solution to the optimal control problems
%                   satisfies all constraints 
%
% Syntax:
%       [res,L_,J_] = CHECKCONSTRAINT(x,u,L,J,Opts)
%
% Description:
%       This function checks if the solution that was determined with the
%       optimal control problems satisfies all constraints. This is
%       necessary because due to the limited computation time the optimal
%       control problems might be terminated before totoal convergence is
%       achieved. Further, the function checks if the cost for the new
%       solution is lower than the cost for the previous solution
%
% Input Arguments:  
%
%       -x:     Center trajectory determined by the optimal control problem
%               (dimension: [nx,N])
%       -u:     Control inputs determined by the optimal control problem
%               (dimension: [nu,N])
%       -L:     Value of the objective function (=cost) of the previous
%               solution
%       -J:     Summed distance of the points from the previous solution to
%               the terminal region 
%       -Opts:  a structure containing following options
%
%           -.dT:           time step. Prediction horizon: Opts.N * Opts.dT 
%           -.Q:            state weighting matrix for the cost function
%                           (center trajectory)
%           -.R:            input weighting matrix for the cost function
%                           (center trajectory)
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope)
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%
% Output Arguments:
%
%       -res:   Flag that specifies if all constraints are satisfied and
%               the cost of the new solution is lower than the one for the 
%               previous solution (0 or 1)
%       -L_:    Value of the objective function (=cost) of the new solution
%       -J_:    Summed distances of the points from the new solution to the
%               terminal region
%
% See Also:
%       reachsetMPC
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

    res = 1;    
    N = size(x,2);
    
    % distance of the points from the terminal region
    A = Opts.termReg.A;
    b = Opts.termReg.b;
    
    b = 1/(1+Opts.alpha) * b;
    dist = zeros(N,1);
    
    for i = 1:N
        dist(i) = max(0,max((A*x(:,i)-b)./b));
    end
    
    J_ = sum(dist);
    
    % value objective function
    L_ = x(:,end)'*Opts.Q*x(:,end);
    for i = 1:N-1
        L_ = L_ + u(:,i)'*Opts.R*u(:,i)*Opts.dT;
    end
    
    % contraction constraint
    if (J_ - J) > - Opts.alpha
        res = 0;
        return;
    end
    
    % constraint final point inside terminal region
    if dist(end) ~= 0
        res = 0;
        return;
    end

    % check if the value of the cost function is decreasing
    if L_ > L
        res = 0;
        return;
    end 
end