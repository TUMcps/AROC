function sOpt = factorScaling(R0,Rfin,H,goalSet,stateCon)
% FACTORSCALING - compute optimal scaling factor for all generators
%
% Syntax:
%       sOpt = factorScaling(R0,Rfin,H,goalSet,stateCon)
%
% Description:
%       This function computes the optimal scaling factors for all
%       generators such that the volume of the initial set is maximized
%       and the final reachable set is fully contained in the goal set. The
%       optimal scaling factors are determined by nonlinear programming
%       using the IPOPT [1] interfaced via CasADi [2].
%
% Input Arguments:
%
%       -R0:        initial set 
%                   (class: zonotope)
%       -Rfin:      final forward reachable set 
%                   (class: polyZonotope)
%       -H:         matrix storing the generator-to-input assignment
%       -goalSet:   set that should be reached
%                   (class: polytope)
%       -stateCon:  set of admissible states
%                   (class: polytope)
%
% Output Arguments:
%
%       -sOpt:     vector containing the optimal scaling factors
%                   (double array of dimension [size(R0.G,2),1])
%
% See Also:
%       safetyNetControl
%
% References:
%       [1] A. Wächter and L. T. Biegler, 'On the Implementation of a 
%           Primal-Dual Interior Point Filter Line Search Algorithm for 
%           Large-Scale Nonlinear Programming', 
%           Mathematical Programming 106(1), pp. 25-57, 2006 
%       [2] J. A. E. Andersson et al., 'CasADi – A software framework 
%           for nonlinear optimization and optimal control', 
%           Mathematical Programming Computation, vol. 11, no. 1, 
%           pp. 1–36, 2019.
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
% Authors:      Moritz Klischat, Niklas Kochdumper, Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % devide zonotope factors into state and input generators
    indInput = find(sum(H,1) ~= 0);
    indAllZero = find(sum(abs(generators(R0)),1) == 0);
    indRem = unique([indInput,indAllZero]);
    
    indState = setdiff(1:length(Rfin.id),indRem);
    
    
    % Constraint Function -------------------------------------------------
    
    % get polynomial zonotope properties
    G = Rfin.G; c = Rfin.c; Grest = Rfin.GI; expMat = Rfin.E;
    
    % extract terms with all even exponents
    temp = prod(ones(size(expMat))-mod(expMat,2),1);
    indEven = find(temp == 1);
    G(:,indEven) = 0.5 * G(:,indEven);
    
    % add a small offset to the constraints
    constrOffset = 1e-3;
    
    % precompute constant parts of the constraint function
    b = goalSet.b - sum(abs(goalSet.A*Grest),2) - constrOffset;
    
    % function handle to the constraint function 
    if representsa(stateCon,'emptySet')
        conFun = @(x) constraintFun(x,expMat,G,c,goalSet.A,b,indEven);
    else
        conFun = @(x) constraintFunStateCon(x,expMat,G,c,goalSet.A,b, ...
                               indEven,stateCon,center(R0),generators(R0));
    end
    
    % Objective Function --------------------------------------------------
    
    % compute number of combinations required for volume computation
    G_ = generators(R0);
    G_ = G_(:,indState);
    
    comb = combinator(size(G_,2),size(G_,1),'c');
    
    if size(comb,1) < 1000      % use exact volume
        
        % normalize generators
        normalize_matrix = diag(1./sum(abs(G_),2));
        G_ =  normalize_matrix * G_;
        
        % function handle for objective function
        objFun_ = @(x) objectivFunExact(x,G_,comb);
        
    else                        % use approximation
        len = sqrt(sum(G_.^2,1));
        objFun_ = @(x) objectiveFunApprox(x,len);
    end
    
    
    % Optimization -------------------------------------------------------- 
               
    nVars = length(Rfin.id);

    % initial guess
    x0 = ones(nVars,1);

    % setup constraint and objective function handles
    ySym = sym('ySym',[nVars,1]);
    conSym = conFun(ySym);
    conFun_ = matlabFunction(conSym,'Vars',{ySym});

    objSym = objFun_(ySym);
    objFun = matlabFunction(objSym,'Vars',{ySym});

    %  IPOPT algorithm settings
    ipoptOptions.ipopt.constr_viol_tol = 1e-6;
    ipoptOptions.ipopt.acceptable_constr_viol_tol = 1e-6;
    ipoptOptions.ipopt.bound_relax_factor = 0;
    ipoptOptions.ipopt.compl_inf_tol = 1e-6;
    ipoptOptions.ipopt.print_level = 0; % no console output of iterations
    ipoptOptions.print_time = 0; % no console output of computation time

    % create solver
    import casadi.*
    y = SX.sym('y',nVars,1);
    nlp = struct('x',y,'f',objFun(y),'g',conFun_(y));
    solver = nlpsol('S','ipopt',nlp,ipoptOptions);

    % bounds on the constraint functions: -inf <= g(y) <= 0
    nCon = length(conFun(x0));
    cl = -inf(nCon,1);
    cu = zeros(nCon,1);

    % bounds on the variables: 
    % -> indices in indState: 0.01 <= s <= inf
    % -> indices in indRem s == 1
    lb = 0.01*ones(length(indState),1);
    lb(indRem) = 1;
    ub = inf(nVars,1);
    ub(indRem) = 1;

    % solve optimization problem
    res = solver('x0',x0,'lbx',lb,'ubx',ub,'lbg',cl,'ubg',cu);

    % check feasibility
    stats = solver.stats();
    if ~stats.success  
        dbstop if warning
        warning('Solution to the optimization problem might be infeasible!')
    end
    
    dbclear if warning

    % extract optimal scaling factors
    sOpt = full(res.x);
                     
end


% Auxiliary Functions -----------------------------------------------------

function c = constraintFun(x,expMat,G,cen,C,d,indEven)
% constraint: containment of the scaled final reachable set in the goal set
% represented by the polytope {x: C*x <= d}

    % compute monomials
    temp = (x*ones(1,size(expMat,2))).^expMat;
    mon = prod(temp,1);

    % compute center vector
    cen = cen + sum(G(:,indEven) * diag(mon(indEven)),2);
    
    % construct the nonlinear constraint function
    c = C*cen + abs(C*G)*mon.' - d;   
end

function c = constraintFunStateCon(x,expMat,G,cen,C,d,indEven,stateCon,c0,G0)
% constraints: 
%   - containment of the scaled final reachable set in the goal set
%     represented by the polytope {x: C*x <= d}
%   - satisfactino of the state constraints (constraint only imposed on the
%     initial set, the goal set is already contained in the set of 
%     admissible states due to the recursive structure  of the algorithm

    % final reachable set is contained in the goal set
    c = constraintFun(x,expMat,G,cen,C,d,indEven);
    
    % intitial set satisfies the state constraints
    cTemp = stateCon.A*c0 + sum(abs(stateCon.A*G0),2) - stateCon.b;  
    c = [c; cTemp];
end

function f = objectivFunExact(x,G,comb)
% objective: exact volume of the scaled initial set

    % initialization
    vol = 0;
    
    % loop over all combinations
    for i = 1:size(comb,1)

        % compute volume
        temp = abs(det(G(:,comb(i,:)))) * prod(x(comb(i,:)));
        vol = vol + temp;   
    end
    
    f = -vol*2^size(G,1);
end

function f = objectiveFunApprox(x,len)
% objecive function: approximate the volume of the zonotope by summing the
% length of its generators
    
    % value of the objecive funtion
    f = -len*x;
end
