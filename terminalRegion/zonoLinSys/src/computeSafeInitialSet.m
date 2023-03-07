function [safeSet,safeInputs] = computeSafeInitialSet(termSet,Param,Opts)
% COMPUTESAFEINITIALSET - compute a safe initial set
%
% Syntax:
%       [safeSet,safeInputs] = COMPUTESAFEINITIALSET(termSet,Param,Opts)
%
% Description:
%       This function computes a safe initial set with corresponding input
%       sets correction sets via optimization, where the objective is to
%       make the initial set as large as possible while steering all states
%       into the previously determined terminal set.
%
% Input Arguments:
%
%       -termSet:    previously computed safe terminal set 
%                    (class: zonotope)
%
%       -Param:      a structure containing the benchmark parameters
%
%           -.U:        set of admissible control inputs 
%                       (class: interval or zonotope)
%           -.W:        set of uncertain disturbances 
%                       (class: interval or zonotope)
%           -.V:        set of measurement errors 
%                       (class: interval or zonotope)
%           -.X:        set of state constraints (class: mptPolytope)
%
%       -Opts:              a structure containing following options
%
%           -.Tdomain:      search domain for the terminal region (class:
%                           interval)
%           -.N:            number of time steps
%           -.genMethod:    method for computing the fixed generator matrix
%                           for the terminal set
%           -.G:            generator matrix for the terminal set (for
%                           Opts.genMethod = 'provided' only)
%           -.costFun:      cost function used for the optimization problem
%
%
% Output Arguments:
%
%       -safeSet:           resulting safe initial set
%       -safeInputs:        cell-array storing the input zonotopes
%
% See Also:
%       computeTermRegZonoLinSys
%
% References:
%       * *[1] Gruber et al. (2021)*, Computing safe sets of linear
%              sampled-data systems, IEEE Control Syst. Lett., vol. 5,
%              no. 2, pp. 385-390
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
% Authors:      Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % check if computation is desired by user
    if strcmp(Opts.costFun,'none')
       safeSet = termSet; 
       safeInputs = cell(1,Opts.N);
       for i = 1:Opts.N
           safeInputs{i} = zonotope(zeros(Opts.nu,size(safeSet.Z,2)));
       end
       return;
    end

    % construct nonscaled fixed generator matrix of safe initial set
    G = computeFixedGeneratorMatrix(Opts,termSet.Z);

    % solve convex optimization problem
    if strcmp(Opts.costFun,'sum')
        [safeSet,safeInputs] = solveLinearProgram(Param,Opts,termSet.Z,G);
    else
        [safeSet,safeInputs] = solveOptProblem(Param,Opts,termSet.Z,G);
    end
end


% Auxiliary Functions -----------------------------------------------------

function G = computeFixedGeneratorMatrix(Opts, safeTermSet)
% compute chosen nonscaled fixed generator matrix

    switch Opts.genMethod
        
        case 'termSet'
            
            % same generator matrix as safe terminal set
            safeTermSet = reduce(zonotope(safeTermSet),'pca',5);
            G = safeTermSet.Z(:,2:end);
            
        case 'sampling2D'
            
            % obtain generator matrix by sampling from unit hypersphere
            % -> only useful for low-dimensional problems
            G = []; cnt = 1;
            Gtmp = generatorsCircle(Opts.nrOfGenerators);
            
            while cnt < Opts.nx
                G = blkdiag(G, Gtmp);
                cnt = cnt + 2;
            end
            
            if mod(Opts.nx,2) ~= 0
                 G = [G; zeros(1,size(G,2))];
                 G = [G, [zeros(Opts.nx-2,size(Gtmp,2));Gtmp]];
            end
            
        case 'provided'
            
            % generator matrix is provided by user
            G = Opts.G;
    end
end

function G = generatorsCircle(nGenerators)
% generate generators by uniformly sampling 2D directions

    % equally spaced angles around top half 2D circle
    theta = linspace(0,pi,1+nGenerators);

    % compute generator matrix
    G = [cos(theta(1:end-1)); sin(theta(1:end-1))];
end

function [safeSet,safeInput] = solveOptProblem(Param,Opts,safeTermSet,G)
% compute a safe initial set via optimization using YALMIP

    % construct optimization variables and parameters
    [scale,initSet,inputSets,RparParam,RtransParam,termSetParam] ...
                    = getVariables(Opts,G,Opts.taylor.Rpar.Z, ...
                                   Opts.taylor.Rtrans.Z,safeTermSet);

    % compute state, input, and terminal constraints
    [stateAndInputCons,Rfin] = getStateInputCons(Param,Opts,initSet, ...
                                         inputSets,RparParam,RtransParam);
                                     
    termCons = getTermCons(Rfin(1:Opts.nx,:), termSetParam);

    % setup optimization problem
    constraints = [stateAndInputCons <= 0, termCons];
    cost = computeScalingGeneratorsCost(Opts, scale);
    
    options = sdpsettings('verbose',0,'allownonconvex',0, ...
                                                    'solver',Opts.solver);

    % solve convex optimization problem
    opt = optimizer(constraints,cost,options,{RparParam,RtransParam, ...
                             termSetParam},{initSet,inputSets,cost,scale});
    
    [res,flag] = opt({Opts.taylor.Rpar.Z,Opts.taylor.Rtrans.Z,safeTermSet});

    % save results to output variables
    [safeSet,safeInput] = saveOptimizationResults(res,flag);
end

function [cons,R] = getStateInputCons(Param,Opts,iniSet,inpSets,Rpar,Rtran)
% get inputs and state constraints from reachability analysis

    % perform reachability analysis
    [Rall,auxTerm] = reachControlledLTISystem(Opts,iniSet,inpSets, ...
                                                               Rpar,Rtran);

    % compute state and input constraints
    cons = stateAndInputConstraints(Param,Opts,Rall,auxTerm,iniSet);

    % final reachable set used for terminal constraint
    R = Rall.Tp{end};
end

function [scale,initSet,inputSets,RparParam,RtransParam,termSetParam] ...
                            = getVariables(Opts,G,Rpar,Rtrans,safeTermSet)
% construct optimization variables and parameter

    % avoid explosion of internally defined variables in YALMIP
    yalmip('clear');

    % state variables
    initialSetCenter = sdpvar(Opts.nx, 1, 'full');
    scale = diag(sdpvar(size(G,2), 1, 'full'));
    initialSetGenerators = G * scale;
    initSet = [initialSetCenter initialSetGenerators];

    % input variables
    inputSets = sdpvar(Opts.nu, 1+size(G,2), Opts.N, 'full');

    % parameters
    RparParam = sdpvar(size(Rpar,1), size(Rpar,2), 'full');
    RtransParam = sdpvar(size(Rtrans,1), size(Rtrans,2), 'full');
    termSetParam = sdpvar(size(safeTermSet,1),size(safeTermSet,2),'full');
end

function terminalConstraints = getTermCons(ZX, ZY)
% linear constraints to check if zonotope ZX is contained in zonotope ZY
% (see Eq. (4) in [1])

    % extract data
    x = ZX(:, 1);               % center
    y = ZY(:, 1);               % center
    X = ZX(:, 2:end);           % generators
    Y = ZY(:, 2:end);           % generators
    nx = size(X, 2);
    ny = size(Y, 2);

    % construct YALMIP variables
    Gamma = sdpvar(ny, nx, 'full');
    beda = sdpvar(ny, 1, 'full');

    % compute terminal constraints
    terminalConstraints = [...
        X == Y*Gamma, ...
        y - x == Y*beda, ...
        norm([Gamma, beda], Inf) <= 1];
end

function cost = computeScalingGeneratorsCost(Opts,scale)
% objective function for optimization

    % select cost function
    if strcmp(Opts.costFun,'sum')
        costFunctionHandle = @sum;
    elseif strcmp(Opts.costFun,'geomean')
        costFunctionHandle = @geomean;
    end

    % maximize -> minimize negation
    cost = -costFunctionHandle(diag(scale));
end

function [safeInitSet,inputSets] = saveOptimizationResults(res,flag)
% extract required values from optimization results 

    % save to output variables
    if flag == 0                            % successfully solved
        
        safeInitSet = zonotope(res{1});

        % iterate over all input sets
        for i = 1:size(res{2},3) 
            inputSets{i} = zonotope(res{2}(:,:,i));
        end
    else
        error(['YALMIP failed to solve the optimization problem! ', ...
               'yalmiperror(exitFlag) = ', yalmiperror(flag)]);
    end
end

function [safeSet,safeInput] = solveLinearProgram(Param,Opts,safeTermSet,G)
% formulate the optimization problem directly as a linear program instead
% of using YALMIP since this significantly speeds up the optimization

    % upper and lower bounds on the states and inputs
    lb = [Opts.Tdomain.inf; Param.U.inf];
    ub = [Opts.Tdomain.sup; Param.U.sup];
    
    % tighten constraints by curvarture terms
    Z = zonotope(interval(lb,ub));
    C = Opts.taylor.FCenter * Z;
    C = zonotope([C.Z, diag(Opts.taylor.FRadius * sum(abs(Z.Z),2))]);
    C = interval(C);
    
    lb = lb - C.inf;
    ub = ub - C.sup;

    % initialize equality and inequality constraint matrices
    dim_x = Opts.nx + size(G,2) + (size(G,2)+1) * Opts.N;
    dim_t = Opts.N * Opts.nxPlusnu * size(G,2);
    
    C1 = sparse(2*Opts.nxPlusnu*size(G,2)*Opts.N,dim_x + dim_t); 
    C2 = sparse(2*size(G,2)*Opts.N,dim_x + dim_t); 
    
    d1 = zeros(2*Opts.nxPlusnu*size(G,2)*Opts.N,1); 
    d2 = zeros(2*size(G,2)*Opts.N,1); 
    
    % indices of state scaling factors and input generators in vector x
    ind_s = Opts.nx+1:Opts.nx+size(G,2);
    ind_u_ = reshape((1:Opts.nu*(size(G,2)+1)),[Opts.nu, size(G,2)+1]);
    ind_u = cell(Opts.N,1);
    
    for i = 1:Opts.N
       start = max(ind_s) + (i-1)*Opts.nu*(size(G,2)+1);
       ind_u{i} = start + (1:Opts.nu * (size(G,2)+1)); 
    end

    % state and input propagation matrix
    A = Opts.XKX(:,1:Opts.nx);
    B = [zeros(Opts.nx,Opts.nu); eye(Opts.nu)];
    
    Aall = A; Ball = {B}; 
    
    if isempty(Opts.KV)
        Dall = zonotope(zeros(Opts.nxPlusnu,1));
    else
        Dall = zonotope(Opts.KV); 
    end
    
    % loop over all time steps
    cnt = 1;
    cnt2 = 1;
    cntAux = dim_x + 1;
    
    for i = 1:Opts.N
       
         % compute state and input constraints
         Atmp = Aall * G;
         Dint = interval(Dall);
         
         for j = 1:Opts.nxPlusnu
             
            ind = [];
             
            for k = 1:size(G,2)
                
               % generators of the initial set
               C1(cnt,ind_s(k)) = Atmp(j,k);
               C1(cnt+1,ind_s(k)) = -Atmp(j,k);
               
               % inputs 
               for l = 1:length(Ball)
                   Btmp = Ball{l};
                   for h = 1:Opts.nu
                      C1(cnt,ind_u{l}(ind_u_(h,k+1))) = Btmp(j,h); 
                      C1(cnt+1,ind_u{l}(ind_u_(h,k+1))) = -Btmp(j,h); 
                   end
               end
               
               % auxiliary variables
               C1(cnt,cntAux) = -1;
               C1(cnt+1,cntAux) = -1;
               
               ind = [ind, cntAux];
               cntAux = cntAux + 1;
               cnt = cnt + 2;
            end
            
            % state and input constraints
            C2(cnt2,ind) = ones(1,length(ind));
            C2(cnt2+1,ind) = ones(1,length(ind));
            
            for l = 1:size(Aall,2)
               C2(cnt2,l) = Aall(j,l);
               C2(cnt2+1,l) = -Aall(j,l);
            end
            
            for l = 1:length(Ball)
               Btmp = Ball{l};
               for h = 1:Opts.nu
                  C2(cnt2,ind_u{l}(ind_u_(h,1))) = Btmp(j,h); 
                  C2(cnt2+1,ind_u{l}(ind_u_(h,1))) = -Btmp(j,h); 
               end
            end
            
            d2(cnt2) = ub(j) - Dint.sup(j);
            d2(cnt2+1) = -(lb(j) - Dint.inf(j));
            
            cnt2 = cnt2 + 2;
         end
        
         % propagate matrices forward in time
         if i < Opts.N
             Aall = Opts.XKX * Opts.taylor.eAt * Aall;

             for j = 1:length(Ball)
                Ball{j} = Opts.XKX * Opts.taylor.eAt * Ball{j}; 
             end
             
             Dall = Opts.XKX * Opts.taylor.eAt * Dall;

             Ball{end+1} = B;
         else
             Aall = Opts.taylor.eAt * Aall;

             for j = 1:length(Ball)
                Ball{j} = Opts.taylor.eAt * Ball{j}; 
             end
             
             Dall = Opts.taylor.eAt * Dall;
         end
         
         Dall = Dall+Opts.taylor.Rtrans+Opts.taylor.Rpar;
         
         if ~isempty(Opts.KV)
            Dall = Dall + zonotope(Opts.KV); 
         end
    end
    
    % scaling factors have to be larger than zero
    C3 = sparse(length(ind_s),dim_x + dim_t);
    d3 = sparse(length(ind_s),1);
    
    C3(:,ind_s) = -eye(length(ind_s));
    
    % bounds on the inputs
    for i = 1:length(ind_u)
       Ctmp = zeros(2*length(ind_u{i}),dim_x+dim_t);
       Ctmp(:,ind_u{i}) = [eye(length(ind_u{i})); -eye(length(ind_u{i}))];
       dtmp = max(abs([lb(Opts.nx+1:end);ub(Opts.nx+1:end)])) ...
                                                    *ones(size(Ctmp,1),1);
       C3 = [C3; Ctmp]; d3 = [d3; dtmp];
    end
    
    % terminal constraint
    cs = safeTermSet(:,1); Gs = safeTermSet(:,2:end);
    
    Dall = project(Dall,1:Opts.nx);
    cd = center(Dall); Gd = generators(Dall);
    
    elem = Opts.nx*(size(G,2)+size(Gd,2)+1);
    Ceq1 = sparse(elem, dim_x+dim_t);
    Ceq2 = sparse(elem,2*size(Gs,2)*(size(G,2)+size(Gd,2)+1));
    deq = zeros(elem,1);
    
    Atmp = Aall*G;
    cnt = 1;
    
    for i = 1:size(G,1)
        
        ind = (1:size(Gs,2)) + size(Gs,2);
        
        for j = 1:size(G,2)+size(Gd,2)
            
            if j <= size(G,2)
                
                % generators of the initial set
                Ceq1(cnt,ind_s(j)) = Atmp(i,j);

                % inputs 
                for l = 1:length(Ball)
                    Btmp = Ball{l};
                    for h = 1:Opts.nu
                       Ceq1(cnt,ind_u{l}(ind_u_(h,j+1))) = Btmp(i,h); 
                    end
                end
                
            else
                deq(cnt) = -Gd(i,j-size(G,2));
            end
            
            % auxiliary variables
            Ceq2(cnt,ind) = -Gs(i,:);
            
            ind = ind + size(Gs,2);
            cnt = cnt + 1;
        end
    end
    
    for i = 1:Opts.nx
       
        % center from the initial set
        for l = 1:size(Aall,2)
           Ceq1(cnt,l) = Aall(i,l);
        end
            
        % center from the inputs
        for l = 1:length(Ball)
           Btmp = Ball{l};
           for h = 1:Opts.nu
              Ceq1(cnt,ind_u{l}(ind_u_(h,1))) = Btmp(i,h); 
           end
        end
        
        % auxiliary variables
        Ceq2(cnt,1:size(Gs,2)) = Gs(i,:);
        
        % offset
        deq(cnt) = cs(i) - cd(i);
        
        cnt = cnt + 1;
    end
    
    elem = size(Gs,2)*(size(G,2)+size(Gd,2)+1);
    C4 = sparse(elem + size(Gs,2),2*elem);
    d4 = sparse(elem + size(Gs,2),1);
    ind = reshape(1:elem,[size(Gs,2),size(G,2)+size(Gd,2)+1]);
    ind_ = ind + elem;
    cnt = 1;
    
    for i = 1:size(Gs,2)
        for j = 1:size(G,2)+size(Gd,2)+1
            C4(cnt,ind(i,j)) = 1;
            C4(cnt,ind_(i,j)) = -1;
            C4(cnt+1,ind(i,j)) = -1;
            C4(cnt+1,ind_(i,j)) = -1;
            cnt = cnt + 2;
        end
    end
    
    for i = 1:size(Gs,2)
        for j = 1:size(G,2)+size(Gd,2)+1
            C4(cnt,ind_(i,j)) = 1;
        end
        d4(cnt) = 1;
        cnt = cnt + 1;
    end
    
    % solver linear program
    C = [C1;C2;C3]; d = [d1;d2;d3];
    C = blkdiag(C, C4); d = [d; d4];
    
    Ceq = [Ceq1, Ceq2];
    
    f = zeros(size(C,2),1);
    f(ind_s) = -ones(length(ind_s),1);
    
    options = optimoptions('linprog','Display','off');

    x = linprog(f,C,d,Ceq,deq,[],[],options);
    
    if isempty(x)
       error('Failed to solve optimization problem!'); 
    end
    
    % construct initial set
    safeSet = zonotope(x(1:Opts.nx),G*diag(x(ind_s))); 
    
    % construct input sets
    safeInput = cell(1,Opts.N);
    
    for i = 1:Opts.N
       Z = zeros(Opts.nu,size(G,2)+1);
       for j = 1:size(Z,1)
           for k = 1:size(Z,2)
               Z(j,k) = x(ind_u{i}(ind_u_(j,k)));
           end
       end    
       safeInput{i} = zonotope(Z);
    end    
end