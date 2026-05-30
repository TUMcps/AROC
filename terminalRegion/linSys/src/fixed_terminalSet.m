function termSet = fixed_terminalSet(optsInternal)
% FIXED_TERMINALSET - compute a safe terminal set, according to [1]
%
% Syntax:
%       safeTermSet = FIXED_TERMINALSET(Param,Opts)
%
% Description:
%       This function computes a safe terminal set by calculating the 
%       reachable set starting from the origin and from the search domain 
%       in parallel until both reachable sets converge to a common set.
%
% Input Arguments:
%
%       -optsInternal:  internal options set by setOptsInternal
%
% Output Arguments:
%
%       -termSet:   resulting safe terminal set 
%                       (class: zonotope)
%
% See Also:
%       ---
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
% Authors:      Felix Gruber, Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    nx = optsInternal.stdDynDims.nx;
    % compute minimal RPI set over-approximation
    Bounds = convergeToMinimalSet(optsInternal);
    N = length(Bounds);
    safeTermSet = Bounds{end};
    
    order = ceil(size(safeTermSet.generators,2)/nx);
    % increase zonotope order until all constraints are satisfied
    for i = 1:order
        % order reduction
        safeTermSet_Test = reduce(safeTermSet,'pca',i);

        % compute constraints + reachable sets for safe terminal set
        con = reachUntilMaxTimeStep(optsInternal,safeTermSet_Test,N);

        % all constraints satisfied? -> success
        if all(con <= 0)
            safeTermSet = safeTermSet_Test;
            break;
        end
    end

    % any constraint not satisfied? -> failure
    if any(0 <= con)
        error(['Failed to find a safe terminal set! Try choosing ', ...
                'different parameters, e.g., feedback matrix K.']);
    end

    termSet.G = generators(safeTermSet);
    termSet.c = center(safeTermSet);
end


% Auxiliary Functions -----------------------------------------------------

function Rbound = convergeToMinimalSet(optsInternal)
% compute reachable sets starting from the origin and from the state bounds
% until both reachable sets converge to a common invariant terminal set
    
    nx = optsInternal.stdDynDims.nx;

    % initialization 
    dist = Inf;
    Rorig = {};
    Rbound = {};

    % initalizing search domain
    Tdomain = initSearchDomain(optsInternal.K,optsInternal.xEq,optsInternal.uEq,optsInternal.Param.U,optsInternal.Param.X);

    % initial state sets:
    Rorig{1} = zonotope(zeros(nx,1)); % -> origin, see [1, Alg. 2, line 2]
    Rbound{1} = zonotope(Tdomain); % -> domain of interset, see [1, Alg. 2, line 3]

    % reachability analysis until convergence to an outer-approximation of 
    % the minimal robust positively invariant set, see [1, Alg. 2, line 4]
    while 1e-2 < dist
        Rtp_orig_previous.X.G = generators(Rorig{end});
        Rtp_orig_previous.X.c = center(Rorig{end});

        Rtp_bound_previous.X.G = generators(Rbound{end});
        Rtp_bound_previous.X.c = center(Rbound{end});

        [Rtp_orig, ~, ~] = fixed_oneStepReachability(Rtp_orig_previous, optsInternal);
        [Rtp_bound, ~, ~] = fixed_oneStepReachability(Rtp_bound_previous, optsInternal);

        % save to struct corresponding to time points
        Rorig{end+1} = zonotope(full(Rtp_orig.X.c), full(Rtp_orig.X.G));
        Rbound{end+1} = zonotope(full(Rtp_bound.X.c), full(Rtp_bound.X.G));
        
        % udpate distance (required for checking convergence)
        dist = hausdorffDistance(Rorig{end}, Rbound{end});
    end
end

function dist = hausdorffDistance(RSmall, RBig)
% compute directed Hausdorff distance w.r.t. infinity norm

    % compute directed Hausdorff distance
    I1 = interval(zonotope(RSmall));
    I2 = interval(zonotope(RBig));
    
    diff = [supremum(I2) - supremum(I1); -(infimum(I2) - infimum(I1))];
    dist = max([0; diff]);
end

function con = reachUntilMaxTimeStep(optsInternal,termSet,maxTimeSteps)
% compute reachable set until the constraints are violated or the maximum
% number of time steps is reached

    % initialization
    con = -1;
    ctr = 0;
    nextR.X.c = center(termSet);
    nextR.X.G = generators(termSet);

    % reachability until constraint violation or max time steps reached
    while (all(con < 0)) && (ctr < maxTimeSteps)
        
        % compute 1-step next reachable sets
        initSet.X.G = nextR.X.G;
        initSet.X.c = nextR.X.c;
        
        [nextR, Rti, utotal] = fixed_oneStepReachability(initSet, optsInternal);

        % Check that the state constraints are fulfilled
        con_inputs = sum(abs(optsInternal.stdConstraints.U.A * utotal.G), 2) - (optsInternal.stdConstraints.U.b - optsInternal.stdConstraints.U.A * utotal.c);
        % Check that the time intervals are satisfying the state
        % constraints
        con_timeInterval = sum(abs(optsInternal.stdConstraints.X.A * Rti.X.G), 2) - (optsInternal.stdConstraints.X.b - optsInternal.stdConstraints.X.A * Rti.X.c);

        % udpate constraints based on next 1-step reachable sets
        con = [con_inputs; con_timeInterval];

        % increment counter
        ctr = ctr + 1;
    end
end