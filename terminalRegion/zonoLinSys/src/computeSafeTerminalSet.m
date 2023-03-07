function safeTermSet = computeSafeTerminalSet(Param,Opts)
% COMPUTESAFETERMINALSET - compute a safe terminal set
%
% Syntax:
%       safeTermSet = COMPUTESAFETERMINALSET(Param,Opts)
%
% Description:
%       This function computes a safe terminal set by calculating the 
%       reachable set starting from the origin and from the search domain 
%       in parallel until both reachable sets converge to a common set.
%
% Input Arguments:
%
%       -Param:     a structure containing the benchmark parameters
%
%           -.U:        set of admissible control inputs 
%                       (class: interval or zonotope)
%           -.W:        set of uncertain disturbances 
%                       (class: interval or zonotope)
%           -.V:        set of measurement errors 
%                       (class: interval or zonotope)
%           -.X:        set of state constraints (class: mptPolytope)
%
%       -Opts:      a structure containing following options
%
%           -.Tdomain:      search domain for the terminal region (class:
%                           interval)
%           -.maxDist:      maximum distance for convergence criterion
%
% Output Arguments:
%
%       -safeTermSet:   resulting safe terminal set (class: zonotope)
%
% See Also:
%       computeSafeInitialSet, computeTermRegZonoLinSys
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

    % compute minimal RPI set over-approximation
    Rbound = convergeToMinimalSet(Param, Opts);
    
    Bounds = zonotope(Rbound{end}(1:Opts.nx,:));
    order = ceil(size(Bounds.generators,2)/Opts.nx);

    % increase zonotope order until all constraints are satisfied
    for i = 1:order
        
        % order reduction
        safeTermSet = reduce(Bounds,Opts.cora.reductionTechnique,i);

        % compute constraints + reachable sets for safe terminal set
        con = reachUntilMaxTimeStep(Param,Opts,safeTermSet,length(Rbound));

        % all constraints satisfied? -> success
        if all(con <= 0)
            break;
        end
    end

    % any constraint not satisfied? -> failure
    if any(0 <= con)
        error(['Failed to find a safe terminal set! Try choosing ', ...
                'different parameters, e.g., feedback matrix K']);
    end
end


% Auxiliary Functions -----------------------------------------------------

function Rbound = convergeToMinimalSet(Param, Opts)
% compute reachable sets starting from the origin and from the state bounds
% until both reachable sets converge to a common invariant terminal set
    
    % initialization 
    dist = Inf;
    con = -1;
    Rorig = {};
    Rbound = {};
    
    % update options
    Opts.N = 1;
    Opts.TpOrTi = 'Tp';

    % initial state sets: 1. origin, 2. bounds intervals
    nextRorig.Tp{1} = zonotope(zeros(Opts.nx,1)).Z;
    nextRbound.Tp{1} = zonotope(Opts.Tdomain).Z;

    % reachability analysis until convergence to invariant set or 
    % constraint violation
    while (Opts.maxDist < dist) && (all(con < 0))
        
        % compute next initial sets -> project onto state dimensions
        initSetOrig = nextRorig.Tp{1}(1:Opts.nx,:);
        initSetBound = nextRbound.Tp{1}(1:Opts.nx,:);

        % compute 1-step next reachable sets
        nextRorig = reachControlledLTISystem(Opts,initSetOrig,[], ...
                                Opts.taylor.Rpar.Z,Opts.taylor.Rtrans.Z);
        nextRbound = reachControlledLTISystem(Opts,initSetBound,[], ...
                                Opts.taylor.Rpar.Z,Opts.taylor.Rtrans.Z);

        % save to struct corresponding to time points
        Rorig{end+1} = nextRorig.Tp{1};
        Rbound{end+1} = nextRbound.Tp{1};

        % udpate distance + constraints based on next 1-step reachable sets
        dist = hausdorffDistance(Rorig{end}, Rbound{end}, Opts.nx);
        con = stateAndInputConstraints(Param,Opts,nextRorig, ...
                                                      [],zeros(Opts.nx,1));
    end
end

function dist = hausdorffDistance(RSmall, RBig, nx)
% compute directed Hausdorff distance w.r.t. infinity norm

    % only interested in states -> extract them
    statesRSmall = RSmall(1:nx,:);
    statesRBig = RBig(1:nx,:);

    % compute directed Hausdorff distance
    I1 = interval(zonotope(statesRSmall));
    I2 = interval(zonotope(statesRBig));
    
    diff = [supremum(I2) - supremum(I1); -(infimum(I2) - infimum(I1))];
    dist = max([0; diff]);
end

function con = reachUntilMaxTimeStep(Param,Opts,termSet,maxTimeSteps)
% compute reachable set until the constraints are violated or the maximum
% number of time steps is reached

    % initialization
    Opts.N = 1;
    con = -1;
    ctr = 0;
    nextR.Tp{1} = termSet.Z;

    % reachability until constraint violation or max time steps reached
    while (all(con < 0)) && (ctr < maxTimeSteps)
        
        % compute next initial sets -> project onto state dimensions
        initSet = nextR.Tp{1}(1:Opts.nx,:);

        % compute 1-step next reachable sets
        [nextR,auxTerm] = reachControlledLTISystem(Opts,initSet,[], ...
                                  Opts.taylor.Rpar.Z,Opts.taylor.Rtrans.Z);

        % udpate constraints based on next 1-step reachable sets
        con = stateAndInputConstraints(Param,Opts,nextR,auxTerm,termSet.Z);

        % increment counter
        ctr = ctr + 1;
    end
end