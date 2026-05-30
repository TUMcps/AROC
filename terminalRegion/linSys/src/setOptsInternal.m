function optsInternal = setOptsInternal(benchmark,Opts,Param)
% SETOPTSINTERNAL - sets up all necessary 'primary' settings needed for the
%                   linSys algorithm
%
% Syntax:
%       optsInternal = setOptsInternal(benchmark,Opts,Param)
%
% Description:
%       This function sets up all necessary 'primary' settings needed for
%       the linSysContainment algorithm. This includes dedicated variables
%       for the dynamics, some quantities needed for reachability analysis
%       (in order to speed up the computation), as well as the system
%       dimension etc.
%       All these 'internal' variables are saved in one big struct,
%       optsInternal. In order to make it easier to track which variables
%       are contained in the struct, and what they mean the actual
%       initialization of the struct will be performed at the very end of
%       this code, in the 'section' called 'Struct Initialization'
%
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
% Authors:      Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    %% Check that Opts and Param are valid, and set default values
    [Opts, Param] = defaultOpts(benchmark,Opts,Param);
    [Opts, Param] = checkOpts_Params(Opts,Param); % We do not yet do the equalDimChecks for
    % the dynamics, this will be done seperately later

    %% Define dynamical system
    [dynamics, sys, stdSys, constraints, stdConstraints, dynDims, stdDynDims] = setUpDynamics(benchmark,Opts,Param);

    
    %% Set up all the parameters needed for the reachability analysis
    reachability = setUpReachability(stdSys,Opts,stdDynDims);

    %% Preparing the fixed directions
    switch Opts.genMethod
        case 'spherical'
            E = ellipsoid(eye(stdDynDims.nx));
            % The two next choices produce the same directions, but in the
            % ellipsoid case, it results in Z containing the unit sphere,
            % which takes much longer to compute
            if strcmp(Opts.terminalRegionSet, 'ellipsoid')
                Z = zonotope(E,'outer:norm',Opts.nGenerators);
            elseif strcmp(Opts.terminalRegionSet, 'zonotope')
                Z = zonotope(E,'inner:norm_bnd',Opts.nGenerators);
            else
                error('Wrong set type for the terminal region.')
            end
            Hfixed = Z.G;
        case 'provided'
            % generator matrix is provided by user
            Hfixed = Opts.G;
        otherwise
            error('Wrong method for selecting the fixed generators.')
    end

    if strcmp(Opts.terminalRegionSet, 'ellipsoid')

    end

    % Setting the size of the scaling coefficients
    switch Opts.terminalRegionSet
        case 'ellipsoid'
            size_s = stdDynDims.nx;
        case 'zonotope'
            size_s = size(Hfixed,2);
        otherwise
            error('Wrong set type for the terminal region.')
    end

    % Using the fixed directions, we can prepare preH and postH, so that
    % the initial set will be defined as X0 = preH * diag(s) * postH

    % initial guess -> LQR; solution of Riccati equation can be used as 
    % (part of) a stabilizing terminal cost
    tpHom = reachability.eAdt;
    tpParU = reachability.Phi * stdSys.B;

    % We may need to extend the weighting matrices Q and R, in case new
    % variables have been added
    stdQ = blkdiag(Opts.Q, speye(stdDynDims.nx - dynDims.nx));
    stdR = blkdiag(Opts.R, speye(stdDynDims.nu - dynDims.nu));

    controllable = rank(full(ctrb(stdSys.A, stdSys.B))) == stdDynDims.nx;
    if ~controllable
        warning('Your system is not controlable, so the algorithm is going to have difficulties finding a good solution, but will still attempt it by skipping the computation of the gain matrix, and setting it to zero (this will not go well), unless you have specified one.')
        L = speye(stdDynDims.nx);
        K = sparse(stdDynDims.nx, stdDynDims.nu);
    else
        [K,P,~] = dlqr(full(tpHom),full(tpParU),full(stdQ),full(stdR));
        L = sqrtm(inv(P));
    end
    if strcmp(Opts.terminalRegionSet, 'ellipsoid')
        preH = L;
        postH = Hfixed;

    else
        preH = L * Hfixed;
        postH = speye(size_s);
    end

    
    % As for the gain matrix, we first ask whether the user has a good
    % guess...
    if ~isempty(Opts.K)
        K = Opts.K;
    else
        % Otherwise, we just take -K from before
        K = -K;
    end

    %% Solver options
    yalmipOptions = sdpsettings('allownonconvex',0,'solver','mosek','verbose',0);

    %% Struct Initialization
    
    % Saving dynamics
    optsInternal.dynamics = dynamics;
    optsInternal.sys = sys;
    optsInternal.stdSys = stdSys;
    optsInternal.dynDims = dynDims;
    optsInternal.stdDynDims = stdDynDims;

    % Saving equilibrium points (only relevant for 'fixed' approach)
    optsInternal.xEq = Opts.xEq;
    optsInternal.uEq = Opts.uEq;

    % Saving reachability quantities
    optsInternal.reachability = reachability;

    % Constraints on state and input
    optsInternal.constraints = constraints;
    optsInternal.stdConstraints = stdConstraints;

    % Saving size_s
    optsInternal.size_s = size_s;

    % Saving preH and postH
    optsInternal.preH = preH;
    optsInternal.postH = postH;

    % Saving terminal region type
    optsInternal.terminalRegionSet = Opts.terminalRegionSet;

    % Saving K
    optsInternal.K = K;

    % Saving solver options
    optsInternal.yalmipOptions = yalmipOptions;

    % Saving the 'filled out' Opts and Param
    optsInternal.Opts = Opts;
    optsInternal.Param = Param;

end


function [Opts,Param] = defaultOpts(benchmark,Opts,Param)
    %% Check that X, U, W are set
    if ~isfield(Param, 'X')
        error('The field Param.X has not been set.');
    elseif ~isfield(Param, 'U')
        error('The field Param.U has not been set.');
    elseif ~isfield(Param, 'W')
        error('The field Param.W has not been set.');
    end

    %% Initialize non-existent fields
    fields = {'N', 'timeStep', 'terminalRegionSet', 'terminalRegionType', 'taylorOrder', 'genMethod', 'nGenerators', 'G', 'K', 'Q', 'R', 'xEq', 'uEq'};
    for i_f = 1:length(fields)
        if ~isfield(Opts, fields{i_f})
            str = ['Opts.', fields{i_f}, ' = [];'];
            eval(str);
        end
    end

    fields = {'F', 'gamma', 'V', 'Y', 'Yexact'};
    for i_f = 1:length(fields)
        if ~isfield(Param, fields{i_f})
            str = ['Param.', fields{i_f}, ' = [];'];
            eval(str);
        end
    end

    %% Set default values for Opts
    Opts.N = setDefaultValues({10},Opts.N);
    Opts.timeStep = setDefaultValues({0.1},Opts.timeStep);
    Opts.terminalRegionSet = setDefaultValues({'zonotope'},Opts.terminalRegionSet);
    Opts.terminalRegionType = setDefaultValues({'implicit'},Opts.terminalRegionType);
    Opts.taylorOrder = setDefaultValues({5},Opts.taylorOrder);

    Opts.xEq = setDefaultValues({zeros([dim(Param.X) 1])},Opts.xEq);
    Opts.uEq = setDefaultValues({zeros([dim(Param.U) 1])},Opts.uEq);

    Opts.genMethod = setDefaultValues({'spherical'},Opts.genMethod);
    inputArgsCheck({{Param.X,'att',{'interval','zonotope','polytope'}}});
    Opts.nGenerators = setDefaultValues({dim(Param.X)},Opts.nGenerators);

    inputArgsCheck({{Param.U,'att',{'interval','zonotope','polytope'}}});

    if isempty(Opts.Q)
        Opts.Q = speye(dim(Param.X));
    end
    if isempty(Opts.R)
        Opts.R = speye(dim(Param.U));
    end

    % We now already construct the dynamics, but don't save it; we need to
    % do it later too, to be consistent
    % get function handle
    str = ['dynamics = @(x,u,w)', benchmark, '(x,u,w);'];
    eval(str);
    
    % get number of states, inputs, and disturbances
    [count,out] = inputArgsLength(dynamics, 3);
    
    % extract variables
    nx = out;
    nu = count(2);
    nw = count(3);
    
    % set up system
    [~, A, B, E, chi] = isLinearModel(dynamics, nx, nu, nw);

    % We can now construct default values for the output equation

    if ~isa(Param.V, 'contSet') && isempty(Param.V)
        Param.V = zonotope(sparse(size(A,1), 1),[]);
    end
    inputArgsCheck({{Param.V,'att',{'interval','zonotope'}}});

    if isempty(Param.F)
        Param.F = sparse(size(A,1), dim(Param.V));
    end
    if isempty(Param.gamma)
        Param.gamma = sparse(size(A,1), 1);
    end

    if ~isa(Param.Y, 'contSet') && isempty(Param.Y)
        Param.Y = polytope(sparse(1, size(A,1)), 0);
    end
    if ~isa(Param.Yexact, 'contSet') && isempty(Param.Yexact)
        Param.Yexact = polytope(sparse(1, size(A,1)), 0);
    end
    
end

function [Opts,Param] = checkOpts_Params(Opts,Param)
    %% Verify validity of Param
    inputArgsCheck({{Param.X,'att',{'interval','zonotope','polytope'}}});
    Param.X = polytope(Param.X);
    %inputArgsCheck({{Param.U,'att',{'interval','zonotope','polytope'}}});
    % This is already done in defaultOpts
    Param.U = polytope(Param.U);
    inputArgsCheck({{Param.W,'att',{'interval','zonotope'}}});
    Param.W = zonotope(Param.W);

    %% Verify validity of Opts
    inputArgsCheck({{Opts.N,'att','numeric',{'scalar','nonnegative','nonnan','finite','integer'}}});
    inputArgsCheck({{Opts.timeStep,'att','numeric',{'scalar','nonnegative','nonnan','finite'}}});
    inputArgsCheck({{Opts.terminalRegionSet,'str',{'zonotope','ellipsoid'}}});
    inputArgsCheck({{Opts.terminalRegionType,'str',{'implicit','fixed'}}});
    if strcmp(Opts.terminalRegionType, 'fixed')
        if strcmp(Opts.terminalRegionSet, 'ellipsoid')
            error("The algorithm Opts.terminalRegionType='fixed' does "+...
                  "not support ellipsoids as terminal regions. Please "+...
                  "check the value of Opts.terminalRegionType.")
        elseif max(abs(Param.F)) > 1e-9
            error("The algorithm Opts.terminalRegionType='fixed' does "+...
                  "not support measurement noise. Please remove the "+...
                  "value in Param.F.")
        end
    end
    inputArgsCheck({{Opts.taylorOrder,'att','numeric',{'scalar','nonnegative','nonnan','finite','integer'}}});

    inputArgsCheck({{Opts.xEq,'att','numeric',{'vector','nonnan','finite'}}});
    inputArgsCheck({{Opts.uEq,'att','numeric',{'vector','nonnan','finite'}}});
    equalDimCheck(Param.X, Opts.xEq);
    equalDimCheck(Param.U, Opts.uEq);

    inputArgsCheck({{Opts.genMethod,'str',{'spherical','provided'}}});
    inputArgsCheck({{Opts.nGenerators,'att','numeric',{'scalar','nonnegative','nonnan','finite','integer'}}});

    if strcmp(Opts.genMethod,'provided')
        inputArgsCheck({{Opts.G,'att','numeric',{'nonnan','finite'}}});
    end

    inputArgsCheck({{Opts.K,'att','numeric',{'nonnan','finite'}}});

    inputArgsCheck({{Opts.Q,'att','numeric',{'nonnan','finite'}}});
    inputArgsCheck({{Opts.R,'att','numeric',{'nonnan','finite'}}});
    % Check if square
    if size(Opts.Q,1) ~= size(Opts.Q,2)
        error("The matrix Opts.Q is not square.")
    end
    if size(Opts.R,1) ~= size(Opts.R,2)
        error("The matrix Opts.R is not square.")
    end
    % Check dims
    equalDimCheck(Param.X, Opts.Q);
    equalDimCheck(Param.U, Opts.R);
    % Check PSD
    if any(eigs(Opts.Q) <= 0)
        error("The matrix Opts.Q is not positive definite.")
    end
    if any(eigs(Opts.R) <= 0)
        error("The matrix Opts.R is not positive definite.")
    end

    inputArgsCheck({{Param.F,'att','numeric',{'nonnan','finite'}}});
    inputArgsCheck({{Param.gamma,'att','numeric',{'vector','nonnan','finite'}}});

    %inputArgsCheck({{Param.V,'att',{'interval','zonotope'}}}); % This is
    %already done in defaultOpts
    Param.V = zonotope(Param.V);
    inputArgsCheck({{Param.Y,'att',{'interval','zonotope','polytope'}}});
    Param.Y = polytope(Param.Y);
    inputArgsCheck({{Param.Yexact,'att',{'interval','zonotope','polytope'}}});
    Param.Yexact = polytope(Param.Yexact);
end

function [dynamics, sys, stdSys, constraints, stdConstraints, dynDims, stdDynDims] = setUpDynamics(benchmark,Opts,Param)
    % get function handle
    str = ['dynamics = @(x,u,w)', benchmark, '(x,u,w);'];
    eval(str);
    
    % get number of states, inputs, and disturbances
    [count,out] = inputArgsLength(dynamics, 3);
    
    % extract variables
    nx = out;
    nu = count(2);
    nw = count(3);
    
    % set up system
    [modelIsLinear, A, B, E, chi] = isLinearModel(dynamics, nx, nu, nw);
    F = Param.F; gamma = Param.gamma;

    sys.A = A; sys.B = B; sys.E = E; sys.chi = chi;
    sys.F = F; sys.gamma = gamma;

    ny = size(A,1); nv = size(F,2);

    dynDims.nx = nx;
    dynDims.ny = ny;
    dynDims.nu = nu;
    dynDims.nw = nw;
    dynDims.nv = nv;
    
    % throw error if chosen system model is not linear or system type is not supported
    if ~ modelIsLinear
        error(['The benchmark "', benchmark, '" is not a linear model']);
    end

    %% Prepare the constraints on the state and input, as well as the disturbances
    constraints.U = polytope(Param.U);
    constraints.W = zonotope(Param.W);
    constraints.V = zonotope(Param.V);

    constraints.X = polytope(Param.X);
    constraints.Y = polytope(Param.Y);
    constraints.Yexact = polytope(Param.Yexact);

    %% Check that the dimensions match
    checkDynamics(sys, constraints);

    %% Set up standard system, where parts of the constraints get absorbed
    %% into the other matrices/vectors

    stdSys = sys;

    % Let's first deal with the centers of constraints.W and
    % constraints.V
    c_W = center(constraints.W);
    c_V = center(constraints.V);

    stdSys.chi = stdSys.chi + stdSys.E * c_W;
    stdSys.gamma = stdSys.gamma + stdSys.F * c_V;

    stdConstraints.X = constraints.X;
    stdConstraints.Y = constraints.Y;
    stdConstraints.Yexact = constraints.Yexact;

    % Now, let's absorb the generator matrices of W and V
    stdSys.E = stdSys.E*generators(constraints.W);
    stdSys.F = stdSys.F*generators(constraints.V);

    stdDynDims.nx = size(stdSys.A,1);
    stdDynDims.ny = size(stdSys.A,1);
    stdDynDims.nu = size(stdSys.B,2);
    stdDynDims.nw = size(stdSys.E,2);
    stdDynDims.nv = size(stdSys.F,2);

    % The constraints for U don't change
    stdConstraints.U = constraints.U;

    % Sparsifying
    stdSys.A = sparse(stdSys.A);
    stdSys.B = sparse(stdSys.B);
    stdSys.E = sparse(stdSys.E);
    stdSys.F = sparse(stdSys.F);
    stdSys.chi = sparse(stdSys.chi);
    stdSys.gamma = sparse(stdSys.gamma);

    % For the 'fixed' method, we also need to check that xEq, uEq is indeed
    % an equilibrium point
    if strcmp(Opts.terminalRegionType, 'fixed')
        if any(abs(A*Opts.xEq + B*Opts.uEq + chi) > 1e-10)
            error('Specified Opts.xEq and Opts.uEq is not a equilibrium point!');
        end
    end

end

function checkDynamics(sys, constraints)
    % Checking the consistency of the linear system itself
    equalDimCheck(constraints.X, sys.A');
    equalDimCheck(constraints.U, sys.B');
    equalDimCheck(constraints.W, sys.E');

    equalDimCheck(constraints.X, sys.A);
    equalDimCheck(constraints.X, sys.B);
    equalDimCheck(constraints.X, sys.E);
    equalDimCheck(constraints.X, sys.chi);

    equalDimCheck(constraints.V, sys.F');

    equalDimCheck(constraints.Y, sys.F);
    equalDimCheck(constraints.Y, sys.gamma);

    % Checking the consistency of the constraints between themselves
    equalDimCheck(constraints.Y, constraints.Yexact);
end

function reachability = setUpReachability(sys,Opts,dims)
    dt = Opts.timeStep;

    % Exponential e^(A * t)
    reachability.eAdt = expm(sys.A * dt);
    % Exponential e^(-A * t)
    reachability.eAminusdt = expm(-sys.A * dt);
    % Compute an approximation of inv(A)(e^(A * t) - I)
    % While doing so, save the powers of A computed on the way
    % Also, do the same with the factors dt^i/factorial(i)
    % Finally, we have to do this for every value of
    % t = dt, 2*dt, ..., Opts.N*dt
    maxit = 1000; % Could be changed, but in practice this is more than enough
    A_powers = cell([1 max(maxit-1,Opts.taylorOrder)]);
    dti_by_facti = cell([1 max(maxit,Opts.taylorOrder+1)]);
    A_powers{1} = speye(dims.nx); % Care: A_powers{k} = A^{k-1}
    dti_by_facti{1} = 1; % Same here
    dti_by_facti{2} = dt;

    % See cora/contDynamics/@linearSys/linearSysDT.m
    currentMatrix = dti_by_facti{2} * A_powers{1};
    Phi = currentMatrix;

    cnt = 2;
    while true
        dti_by_facti{cnt+1} = dti_by_facti{cnt} * dt/cnt;
        A_powers{cnt} = A_powers{cnt-1} * sys.A;
        currentMatrix = dti_by_facti{cnt+1} * A_powers{cnt};
        Phi = Phi + currentMatrix;
        cnt = cnt + 1;
        if all(all(abs(currentMatrix) < eps)) || cnt > maxit
            break;
        end
    end

    % Saving Phi
    reachability.Phi = Phi;

    % Filling up A_powers and dti_by_facti, in case not enough factors have
    % been computed
    if cnt-1 <= Opts.taylorOrder
        while cnt-1 <= Opts.taylorOrder
            A_powers{cnt} = A_powers{cnt-1} * sys.A;
            dti_by_facti{cnt+1} = dti_by_facti{cnt} * dt/cnt;
            cnt = cnt + 1;
        end
    end
    % Cut out the parts that were not computed
    A_powers = A_powers(1:cnt-1);
    dti_by_facti = dti_by_facti(1:cnt);

    reachability.A_powers = A_powers;
    reachability.dti_by_facti = dti_by_facti;

    % From now on, we use the same notation as in 
    % "Fully Automated Verification of Linear Systems Using Inner and Outer
    %  Approximations of Reachable Sets", by Wetzlinger et al.
    % Computing the error interval matrix E
    E_width = expm(abs(sys.A) * dt);
    for i = 0:Opts.taylorOrder
        E_width = E_width - abs(sys.A)^i  * dti_by_facti{i+1};
    end
    E_interval = interval(-E_width, E_width);

    % Computing the interval matrix F
    Fsum_pos=sparse(dims.nx,dims.nx);
    Fsum_neg=sparse(dims.nx,dims.nx);

    for i=2:Opts.taylorOrder
        %compute factor
        exp1=-i/(i-1); exp2=-1/(i-1);
        factor=(i^exp1-i^exp2)*dti_by_facti{i+1};

        %init Apos, Aneg
        Apos=sparse(dims.nx,dims.nx);
        Aneg=sparse(dims.nx,dims.nx);

        %obtain positive and negative parts
        pos_ind = (A_powers{i+1})>0;
        neg_ind = (A_powers{i+1})<0;

        Apos(pos_ind) = A_powers{i+1}(pos_ind);
        Aneg(neg_ind) = A_powers{i+1}(neg_ind);

        %compute powers; factor is always negative
        Fsum_pos=Fsum_pos + factor*Aneg;
        Fsum_neg=Fsum_neg + factor*Apos;
    end
    
   F_interval = interval(Fsum_neg, Fsum_pos);
   F_interval = F_interval + E_interval;

   % Computing the interval matrix G
   %initialize G
    Gsum_pos=sparse(dims.nx,dims.nx);
    Gsum_neg=sparse(dims.nx,dims.nx);

    for i=2:Opts.taylorOrder+1
        %compute factor
        exp1=-i/(i-1); exp2=-1/(i-1);
        factor=(i^exp1-i^exp2)*dti_by_facti{i+1};

        %init Apos, Aneg
        Apos=sparse(dims.nx,dims.nx);
        Aneg=sparse(dims.nx,dims.nx);

        %obtain positive and negative parts
        pos_ind = (A_powers{i})>0;
        neg_ind = (A_powers{i})<0;

        Apos(pos_ind) = A_powers{i}(pos_ind);
        Aneg(neg_ind) = A_powers{i}(neg_ind);

        %compute powers; factor is always negative
        Gsum_pos=Gsum_pos + factor*Aneg;
        Gsum_neg=Gsum_neg + factor*Apos;
    end
    
    G_interval = interval(Gsum_neg, Gsum_pos);
    G_interval = G_interval + E_interval * dt;

    % Saving interval matrices
    reachability.E_interval = E_interval;
    reachability.F_interval = F_interval;
    reachability.G_interval = G_interval;

    % The particular solution due to the disturbances is a bit trickier.
    % Basically, it can be written as
    % sum_{j=0}^Opts.taylorOrder [A^k*dti^k*E_/(k+1)!] 
    %            + dt*E_interval * M([-1,1]) * E_
    % The notation M([-1,1]) denotes the space of matrices with
    % coefficients in [-1,1], and E_ = E * [-1,1]^nw
    %
    % First need to compute E_ (or rather, the generator matrix thereof)
    me = size(sys.E, 2);
    E_ = sys.E * zonotope(interval(-ones([me 1]), ones([me 1])));
    E_ = generators(E_);

    % The first sum is the easiest to construct:
    tpParWapprox = cell([1 Opts.taylorOrder+1]);
    for j = 0:Opts.taylorOrder
        tpParWapprox{j+1} = A_powers{j+1} * dti_by_facti{j+2} * E_;
    end

    tpParW = cat(2, tpParWapprox{:});

    % tpParWerror is also easy to compute:
    tpParWerror = dt * E_interval;

    % It now remains to over-approximate M([-1,1]) * E * [-1,1]^n. This can
    % be done as mentioned in
    % "Fully-Automated Verification of Linear Systems Using Reachability
    % Analysis with Support Functions", Wetzlinger et al.
    P_dist = [tpParW diag(rad(tpParWerror) * sum(abs(E_),2))];

    % Great, we have the influence of the disturbance; can we simplify it?
    Z_dist = zonotope(sparse(dims.nx,1), P_dist);
    Z_dist = compact(Z_dist);
    P_dist = generators(Z_dist);

    reachability.P_dist = P_dist;
    reachability.m_P_dist = size(P_dist,2);

    reachability.taylorOrder = Opts.taylorOrder;

end