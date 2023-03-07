function s = factorScaling(R0,Rfin,H,C,d,stateCon)
% FACTORSCALING - compute optimal scaling factor for all generators
%
% Syntax:
%       s = FACTORSCALING(R0,Rfin,H,C,d,stateCon)
%
% Description:
%       This function computes the optimal scaling factors for all
%       generators such that the volume of the initial set is maximized
%       and the final reachable set is fully contained in the goal set. The
%       optimal scaling factors are determined by nonlinear programming
%       using the "fmincon" algorithm.
%
% Input Arguments:
%
%       -R0:        initial set (class: zonotope)
%       -Rfin:      final forward reachable set (class: polyZonotope)
%       -H:         matrix storing the generator-to-input assignment
%       -C:         matrix for the inequality constraint C*x <= d
%       -d:         vector for the inequality constraint C*x <= d
%       -stateCon:  struct containing state constraint parameter
%
% Output Arguments:
%
%       -s:     vector with the optimal scaling factor for each generator
%
% See Also:
%       safetyNetControl
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
% Authors:      Moritz Klischat, Niklas Kochdumper
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
    G = Rfin.G; c = Rfin.c; Grest = Rfin.Grest; expMat = Rfin.expMat;
    
    % extract terms with all even exponents
    temp = prod(ones(size(expMat))-mod(expMat,2),1);
    indEven = find(temp == 1);
    G(:,indEven) = 0.5 * G(:,indEven);
    
    % add a small offset to the constraints
    constrOffset = 1e-3;
    
    % precompute constant parts of the constraint function
    d = d - sum(abs(C*Grest),2) - constrOffset;
    
    % function handle to the constraint function 
    if isempty(stateCon)
        conFun = @(x) constraintFun(x,expMat,G,c,C,d,indState,indEven);
    else
        conFun = @(x) constraintFunStateCon(x,expMat,G,c,C,d,indState, ...
                               indEven,stateCon,center(R0),generators(R0));
    end
    
    % lower bound
    lb = 0.01*ones(length(indState),1);
    
    
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
               
    % solve the nonlinear optimization problem with "active-set" algorithm
    options = optimoptions('fmincon','Algorithm','active-set', ...
                           'Display','iter','MaxFunctionEvaluations',20000);
                       
    x0 = ones(length(indState),1);

    sOpt = fmincon(objFun_,x0,[],[],[],[],lb,[],conFun,options);
            
    % check if the constraints are satisfied
    c = conFun(sOpt);
    
    if max(c) > 1e-6
        
        options = optimoptions('fmincon','Algorithm','interior-point', ...
                               'Display','iter','MaxIter',100000, ...
                               'MaxFunEvals',100000);

        sOpt = fmincon(objFun_,x0,[],[],[],[],lb,[],conFun,options);        
    end

    % construct final vector of scaling factors
    s = ones(length(indRem) + length(indState),1);
    s(indState) = sOpt;
               
end


% Auxiliary Functions -----------------------------------------------------

function [c,ceq] = constraintFun(x,expMat,G,cen,C,d,indState,indEven)
% compute the amount of constraint violation

    % initialize variables
    s = ones(size(expMat,1),1);
    s(indState) = x;

    % compute monomials
    temp = (s*ones(1,size(expMat,2))).^expMat;
    mon = prod(temp,1);

    % compute center vector
    cen = cen + sum(G(:,indEven) * diag(mon(indEven)),2);
    
    % construct the nonlinear constraint function
    c = C*cen + abs(C*G)*mon' - d;   
    
    ceq = [];
end

function [c,ceq] = constraintFunStateCon(x,expMat,G,cen,C,d,indState, ...
                                         indEven,stateCon,c0,G0)
% compute the amount of constraint violation under consideration of state
% constraints

    % constraint that the final reachable set is inside the terminal region
    [c,ceq] = constraintFun(x,expMat,G,cen,C,d,indState,indEven);
    
    % constraint that the intitial set satisfies the state constraints
    cTemp = stateCon.A*c0 + sum(abs(stateCon.A*G0),2) - stateCon.b;  
    c = [c; cTemp];
end

function [f, g] = objectivFunExact(x,G,comb)
% computes the value of the objecive function by considering the exact
% volume of a zonotope

    % initialization
    vol = 0;
    grad = zeros(length(x),1);
    
    % loop over all combinations
    for i = 1:size(comb,1)
        
        % compute volume
        temp = abs(det(G(:,comb(i,:)))) * prod(x(comb(i,:)));
        vol = vol + temp;   
        
        % compute gradient
        grad(comb(i,:)) = grad(comb(i,:)) + temp./x(comb(i,:));
    end
    
    f = -vol*2^size(G,1);
    g = -grad*2^size(G,1);

end

function [f,g] = objectiveFunApprox(x,len)
% computes the value of the objecive function by considering an
% approximation of the volume of a zonotope
    
    % value of the objecive funtion
    f = -len*x;
    
    % value of the gradient
    g = -len;
end