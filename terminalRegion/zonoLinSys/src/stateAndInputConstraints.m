function con = stateAndInputConstraints(Param,Opts,R,auxTerm,initSet)
% STATEANDINPUTCONSTRAINTS - compute state and input constraints
%
% Syntax:
%       con = STATEANDINPUTCONSTRAINTS(Param,Opts,R,auxTerm,initSet)
%
% Description:
%       This function computes augmented state and input constraints for
%       the whole time horizon Opts.N. The constraints are fulfilled if the
%       output variable con <= 0.
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
%           -.TpOrTi:       consider reachable sets at time point only
%                           (Opts.TpOrTi = 'Tp') or also the time interval 
%                           reachable sets (Opts.TpOrTi = 'Ti')
%           -.Tdomain:      search domain for the terminal region (class:
%                           interval)
%           -.N:            number of time steps
%
%       -R:     	cell-array containing overall reachable sets
%       -auxTerm:   cell-array containing auxiliary terms for the curvature
%                   of the time interval reachable set
%       -initSet:   initial set
%
% Output Arguments:
%
%       -con:   vector storing the resulting constraints
%
% See Also:
%       reachControlledLTISystem, computeTermRegZonoLinSys
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

    % initialization
    minBounds = [Opts.Tdomain.inf; Param.U.inf];
    maxBounds = [Opts.Tdomain.sup; Param.U.sup];
    con = [];

    % iterate over all time steps
    for i = 1:Opts.N

        % select time point or time interval reachable set
        if strcmp(Opts.TpOrTi, 'Tp')
            Ri = R.Tp{i};
        elseif strcmp(Opts.TpOrTi, 'Ti')
            Ri = R.Ti{i};
        end
        
        % compute absolute value for the sum of generators
        c = Ri(:,1);
        G = Ri(:,2:end);
        sumAbs = sum(abs(G),2);

        % time interval -> also consider abs term (always positive)
        if strcmp(Opts.TpOrTi, 'Ti') 
            sumAbs = sumAbs + sum(auxTerm{i}, 2);
        end

        % compute min and max constraints + append them
        con = appendConstraints(con, c, sumAbs, minBounds, maxBounds);
    end

    % time point -> also consider initial set (only state dimension)
    if strcmp(Opts.TpOrTi, 'Tp')
        projMinBounds = minBounds(1:Opts.nx,:);
        projMaxBounds = maxBounds(1:Opts.nx,:);
        c = initSet(:,1);
        sumAbs = sum(abs(initSet(:,2:end)),2);
        con = appendConstraints(con,c,sumAbs,projMinBounds,projMaxBounds);
    end
end


% Auxiliary Functions -----------------------------------------------------

function con = appendConstraints(con,c,sumAbs,minBounds,maxBounds)
% append constraints by upper and lower bound

    % compute min and max constraints
    minCon = -c + sumAbs + minBounds;
    maxCon =  c + sumAbs - maxBounds;

    % concatenate constraints
    minAndMaxConstraints = [minCon; maxCon];

    % append constraints
    con = [con; minAndMaxConstraints];
end