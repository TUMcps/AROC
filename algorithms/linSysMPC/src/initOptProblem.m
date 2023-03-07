function [opt,optInit] = initOptProblem(Opts)
% INITOPTPROBLEM - initialize optimal control problem for linear MPC
%
% Syntax:
%       [opt,optInit] = INITOPTPROBLEM(Opts)
%
% Description:
%       This function constructs the optimal control problem for linear
%       model predictive control as descibed in Sec. IV.C in [1] using 
%       the YALMIP optimizer.
%
% Input Arguments:
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.N:            number of time-steps until safe terminal set is
%                           reached starting in safe initial set
%           -.K:            feedback matrix
%           -.xf:           desired final state
%           -.termReg:      terminal region
%           -.XKX:          feedback matrix for the extended system
%                           dynamics
%           -.taylor:       struct containing the propagation matrices for
%                           the linear system
%           -.U:            input set
%           -.X:            set of state constraints
%           -.nx:           number of states
%           -.nx:           number of inputs
%
% Output Arguments:
%
%       -opt:       resulting YALMIP optimizer object
%       -optInit:   resulting YALMIP optimizer object for the initial
%                   optimal control problem
%
% See Also:
%       linSysMPC
%
% References:
%       * *[1] F. Gruber and M. Althoff, “Scalable robust model predictive
%              control for linear sampled-data systems," CDC 2019
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
% Authors:      Max Beier, Laura Lützow, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % avoid explosion of internally defined variables in YALMIP
    yalmip('clear');

    % variables
    x0 = sdpvar(Opts.nx,1,'full');
    u = sdpvar(Opts.nu,Opts.N,'full');
    u0 = sdpvar(Opts.nu,1,'full');
    p = sdpvar(Opts.N+1,1,'full');
    J = sdpvar(1,1,'full');

    % compute reachable set
    [R,Rdist,auxTerm,curv] = reachableSet(x0,u,Opts);
    
    % construct state and input constraints
    con1 = stateAndInputConstraints(R,Rdist,auxTerm,curv,Opts);

    % construct contraction constraint
    con2 = contractionConstraint(R,Rdist,p,Opts);
    con = [con1; con2];

    % cost function
    xf = R{end} + curv{end} + center(Rdist{end});
    cost = sum(abs(Opts.Q*(xf(1:Opts.nx) - Opts.xf)));
    
    for i = 1:Opts.N
        cost = cost + Opts.dt * sum(abs(Opts.R * u(:,i)));
    end

    cost = cost + 1e-6*sum(p);
    
    % initialize optimizer
    options = sdpsettings('verbose',0,'allownonconvex',0);

    optInit = optimizer(con,cost,options,{x0},{u,p});

    con_ = [con; u(:,1) == u0];
    con_ = [con_; sum(p) - J + Opts.alpha <= 0];

    opt = optimizer(con_,cost,options,{x0,u0,J},{u,p});
end


% Auxiliary Functions -----------------------------------------------------

function [R,Rdist,auxTerm,curv] = reachableSet(x0,u,Opts)
% comptue reachable set of the controlled system

    % initialization
    R = cell(Opts.N,1);
    Rdist = cell(Opts.N,1);
    auxTerm = cell(Opts.N,1);
    curv = cell(Opts.N,1);

    % compute reachable set
    R{1} = [x0; zeros(Opts.nu, 1)];
    Rdist{1} = Opts.V;

    for i = 1:Opts.N
        
        % update control inputs
        Rinit = Opts.XKX*R{i};
        Rinit(Opts.nx+1:Opts.nx+Opts.nu,:) = ...
              Rinit(Opts.nx+1:Opts.nx+Opts.nu,:) + u(:,i) - Opts.K*Opts.xf;
        
        % reachable set propagation
        R{i+1} = Opts.taylor.eAt*Rinit;
        Rdist{i+1} = Opts.taylor.eAt*Opts.XKX*(Rdist{i} + Opts.V) + ...
                                    Opts.taylor.Rpar + Opts.taylor.Rtrans;
        
        % compute curvature
        curv{i} = Opts.taylor.FCenter * Rinit;
        auxTerm{i} = auxiliaryTerm(Opts.taylor.FRadius, Rinit);
    end
end

function result = auxiliaryTerm(X, Y)
% compute auxiliary term required for multiplication of zonotope and 
% interval matrix (see "contSet\@zonotope\intervalMultiplication" in CORA)

    absY = abs(Y);
    XTimesSumAbsY = X * sum(absY, 2);
    result = diag(XTimesSumAbsY);
end

function con = stateAndInputConstraints(R,Rdist,auxTerm,curv,Opts)
% compute state and input constraints
    
    con = [];

    % construct constraints for extended system dynamics in Eq. (4) in [1]
    if isfield(Opts,'X') && ~isempty(Opts.X)
        poly = cartProd(Opts.X,mptPolytope(Opts.U));
    else
        tmp = mptPolytope(Opts.U);
        A = [zeros(size(tmp.P.A,1),Opts.nx), tmp.P.A];
        poly = mptPolytope(A,tmp.P.b);
    end

    % loop over all reachable sets
    for i = 2:Opts.N+1
        
        % subtract reachable set due to disturbances (see Eq. (9) in [1])
        poly_ = minkDiff(poly,Rdist{i},'inner');
        
        % construct constraint
        if i < Opts.N+1
            con = [con; poly_.P.A * (R{i} + curv{i}) + ...
                        sum(abs(poly_.P.A)*auxTerm{i},2) - poly_.P.b <= 0];
        end
                                
        con = [con; poly_.P.A * (R{i} + curv{i-1}) + ...
                      sum(abs(poly_.P.A)*auxTerm{i-1},2) - poly_.P.b <= 0];
    end   
end

function con = contractionConstraint(R,Rdist,p,Opts)
% compute the contraction constraints

    con = [];

    % terminal region properties
    termReg = Opts.termReg + (-Opts.xf);
    A = termReg.P.A;
    b = termReg.P.b;
    b = 1/(1 + Opts.alpha) * b;

    % loop over all reachable sets
    for i = 1:Opts.N+1

        % current reachable set
        x = R{i} + center(Rdist{i});
        G = generators(Rdist{i});
        x = x(1:Opts.nx); G = G(1:Opts.nx,:);

        % required scaling factors for each halfspace
        tmp = diag(1./b)*(A*(x-Opts.xf) + sum(abs(A*G),2) - b);

        % construct constraints to get maximum scaling factor
        con = [con; p(i) >= 0];

        for j = 1:length(tmp)
            con = [con; tmp(j) - p(i) <= 0];
        end
    end

    % final reachable set has to be inside terminal region
    for i = 1:length(tmp)
        con = [con; tmp(i) <= 0];
    end
end