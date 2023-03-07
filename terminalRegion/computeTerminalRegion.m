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
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -alg:          string specifying the algorithm that is used to
%                      calculate the terminal region. The available
%                      algorithms are 'subpaving' (see [1]) and 
%                      'zonoLinSys' (see [2])
%       -Param:        a structure containing the benchmark parameters
%
%           -.U:       set of admissible control inputs (class: interval)
%           -.W:       set of uncertain disturbances (class: interval)
%
%       -Opts:         a structure containing the algorithm settings
%
% Output Arguments:
%
%       -T:     object of class terminalRegion
%
% See Also:
%       terminalRegion, termRegSubpaving, termRegZonoLinSys
%
% References:
%       * *[1] El-Guindy et al. (2017)*, Estimating the region of 
%              attraction via forward reachable sets, ACC 2017
%       * *[2] Gruber et al. (2021)*, Computing safe sets of linear 
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
% Authors:      Niklas Kochdumper, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    if strcmp(alg,'subpaving')
        T = compTermRegSubpaving(benchmark,Param,Opts);
    elseif strcmp(alg,'zonoLinSys')
        T = computeTermRegZonoLinSys(benchmark,Param,Opts);
    elseif strcmp(alg,'nonlinSysLinApproach')
        T = computeTermRegNonlinSysLinApproach(benchmark,Param,Opts);
    else
        error('Wrong value for input argument "alg"!')
    end
end