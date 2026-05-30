function [flagContains,vars] = zonoContainment(Zin,Zout,tol)
% ZONOCONTAINMENT - solves the zonotope containment problem.
%
% Syntax:
%       [isIn,vars] = zonoContainment(Zin,Zout,tol)
%
% Description:
%       Given two zontopes Zin and Zout, this function checks whether Zin
%       is contained in Zout using the conditions from [1, Corrolary 4]. In
%       contrast to the function @zonotope/contains from the CORA toolbox,
%       the values of the auxiliary varibales Gamma and gamma are available
%       as well, e.g., to set an initial guess for nonlinear programming.
%
% Input Arguments:
%       -Zin:       inbody
%                   (class: zonotope)
%       -Zout:      circumbody
%                   (class: zonotope)
%       -tol:       tolerance for the containment check
%                   (positive scalar)
%
% Output Arguments:
%       -flagContains:      true/false
%       -vars:              structure storing the auxiliary variables:
%                           -.Gamma, gamma from [1, Eq. (15)]
%                           -.slack: magnitude of the violation of the norm
%                           condition in [1, Eq. (15)]
%
% See Also:
%       -
% 
% References:
%       [1] S. Sadraddini and R. Tedrake, "Linear Encodings for Polytope 
%           Containment Problems," IEEE Conference on Decision and Control,
%           2019, pp. 4367-4372.
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    if ~exist('tol','var') || tol < 0
        tol = 1e-6;
    end

    % extract the generators
    Gin = Zin.G;
    Gout = Zout.G;
    nGenIn = size(Gin,2);
    nGenOut = size(Gout,2);
    % Extract the centers
    cIn = Zin.c;
    cOut = Zout.c;

    % add the difference of the centers to the generators of the inbody;
    % one can show that this yields an equivalent optmization problem
    % Gin = [Gin cIn-cOut];
    Gin = [Gin cOut-cIn];
   
    % extract the number of generators
    mIn = nGenIn + 1;
    mOut = nGenOut;
    
    % extract the dimension
    n = size(Gin,1);
    
    % sparsifying
    Gin = sparse(Gin);
    Gout = sparse(Gout);
    
    % setting up constraints for the linear program
    % Gamma - GammaAux <= 0
    A_positive = [speye(mIn*mOut) -speye(mIn*mOut) sparse(mIn*mOut,1)];
    b_positive = sparse(mIn*mOut,1);
    % -Gamma - GammaAux <= 0
    A_negative = [-speye(mIn*mOut) -speye(mIn*mOut) sparse(mIn*mOut,1)];
    b_negative = sparse(mIn*mOut,1);
    % row-wise sum of GammaAux <= w
    A_rowwiseSum = [sparse(mOut,mIn*mOut) kron(sparse(ones([1 mIn])),speye(mOut)) -sparse(ones([mOut 1]))];
    b_rowwiseSum = sparse(mOut,1);
    
    % building the inequality conditions
    Aineq = [A_positive;A_negative;A_rowwiseSum];
    bineq = [b_positive;b_negative;b_rowwiseSum];
    
    % Gout * Gamma = Gin
    Aeq = [kron(speye(mIn), Gout) sparse(n*mIn,mIn*mOut) sparse(n*mIn,1)];
    beq = Gin(:);
    
    % defining cost
    cost = [sparse(mIn*mOut,1); sparse(mIn*mOut,1); 1];
    
    % Problem structure for CORA linprog
    problem.f = cost;
    problem.Aeq = Aeq;
    problem.beq = beq;
    problem.Aineq = Aineq;
    problem.bineq = bineq;
    problem.lb = [];
    problem.ub = [];

    % solve optimization problem
    [xOpt,fVal,~,~,~] = CORAlinprog(problem);

    % extract variables
    xOpt_ = reshape(xOpt(1:(mIn)*nGenOut),nGenOut,nGenIn+1);
    vars.Gamma = xOpt_(:,1:nGenIn);
    vars.gamma = xOpt_(:,end);
    vars.slack = max(0,fVal-1); % violation of norm condition in [1, Eq. (15)]

    % check whether the norm condition in [1, Eq. (15)] is satisfied
    if fVal <= 1+tol
        flagContains = 1;
    else
        flagContains = 0;
    end
end
