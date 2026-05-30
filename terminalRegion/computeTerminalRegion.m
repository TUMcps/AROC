function T = computeTerminalRegion(benchmark,alg,Param,Opts)
% COMPUTETERMINALREGION - compute a terminal region
%
% Syntax:
%       T = COMPUTETERMINALREGION(benchmark,alg,Param,Opts)
%
% Description:
%       This function first computes a terminal controller that stabilizes
%       the system around the specified equilibrium, and then computes a 
%       terminal region for the contolled system with the selected
%       algorithm.
%
% Input Arguments:
%
%       -benchmark:     name of the considered benchmark model (see
%                       "aroc/benchmarks/dynamics/...")
%       -alg:           string specifying the algorithm that is used to
%                       calculate the terminal region. The available
%                       algorithms are 'subpaving' (see [1]), 'linSys' (see 
%                       [2],[3]) and 'nonlinSysOpt' (see [4],[5])
%       -Param:         a struct containing the benchmark parameters
%           -.X:
%           -.U:        set of admissible control inputs 
%                       (class: interval)
%           -.W:        set of disturbances 
%                       (class: interval / zonotope)
%           –.V:        set of measurement errors V 
%                       (class: interval / zonotope)
%           –.X:        set of state constraints X 
%                       (class: polytope)
%
%       -Opts:          a struct containing the algorithm settings
%
% Output Arguments:
%
%       -T:     object of class terminalRegion
%
% See Also:
%       terminalRegion, termRegSubpaving, termRegZonoLinSys,
%       termRegNonlinSsyOptApproach
%
% References:
%       * *[1]  El-Guindy et al., "Estimating the region of attraction via 
%               forward reachable sets", 2017, American Control Conference
%       * *[2]  Gruber et al., "Computing safe sets of linear sampled-data 
%               systems", 2021, IEEE Control Syst. Lett., vol. 5, 
%               no. 2, pp. 385-390
%       * *[3]  A. Kulmburg et al., "Approximability of the Containment 
%               Problem for Zonotopes and Ellipsotopes", 2025, IEEE 
%               Transactions on Automatic Control, vol. 70, no. 12, 
%               pp. 8104-8119.
%       * *[4]  L. Schäfer et al., "Scalable Computation of Robust Control 
%               Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%               on Automatic Control, vol. 69, no. 2, pp. 755-770.
%       * *[5]  L. Schäfer and M. Althoff, "Computing Robust Control 
%               Invariant Sets of Nonlinear Systems Using Polynomial 
%               Controller Synthesis," 2024 American Control Conference, 
%               pp. 4162-4169.
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
% Authors:      Niklas Kochdumper, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    if strcmp(alg,'subpaving')
        T = compTermRegSubpaving(benchmark,Param,Opts);
    elseif strcmp(alg,'linSys')
        T = computeTermReglinSys(benchmark,Param,Opts);
    elseif strcmp(alg,'nonlinSysOpt')
        T = computeTerminalRegionNonlinSysOpt(benchmark,Param,Opts);
    else
        error('Wrong value for input argument "algorithm"!')
    end
end
