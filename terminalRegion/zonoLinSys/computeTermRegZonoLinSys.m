function T = computeTermRegZonoLinSys(benchmark,Param,Opts)
% COMPUTETERMREGZONOLINSYS - compute terminal region with linear system
%                            approach using zonotopes
%
% Syntax:
%       T = COMPUTETERMREGZONOLINSYS(benchmark,Param,Opts)
%
% Description:
%       This function computes a terminal region with the linear system
%       approach in [1].
%
% Input Arguments:
%
%       -benchmark:  name of the considered benchmark model (see
%                    "aroc/benchmarks/dynamics/...")
%
%       -Param:      a structure containing the benchmark parameters
%
%           -.U:     set of admissible control inputs (class: interval)
%           -.W:     set of uncertain disturbances (class: interval)
%           -.X:     set of state constraints (class: interval)
%
%       -Opts:              a structure containing following options
% 
%           -.Tdomain:      search domain for the terminal region (class:
%                           interval)
%           -.xEq:          states for the equibrium point of the system
%           -.uEq:          control inputs for the equilibrium point of the
%                           system
%           -.timeStep:     time step size for the sampled data controller
%                           [positive scalar]
%           -.N:            number of time steps
%                           [{10}, positive integer]
%           -.K:            feedback matrix for the terminal contorller
%           -.Q:            state weighting matrix for the LQR approach
%                           applied to determine the terminal controller
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the LQR approach
%                           applied to determine the terminal controller
%                           [{zeros(nu)} / positive-definite square matrix]
%           -.maxDist:      maximum distance for convergence criterion
%                           [{1e-2} / positive scalar]
%           -.genMethod:    method for computing the fixed generator matrix
%                           for the terminal set
%                           [{'termSet'} / 'sampling2D', 'provided']
%           -.G:            generator matrix for the terminal set (for
%                           Opts.genMethod = 'provided' only)
%           -.costFun:      cost function used for the optimization problem
%                           [{'geomean'} / 'sum', 'none']
%
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{4} / positive integer]
%           -.cora.zonotopeOrder:       maximum zonotope order
%                                       [{150} / positive integer]
%
% Output Arguments:
%
%       -T:     object of class termRegZonoLinSys
%
% See Also:
%       terminalRegion, termRegZonoLinSys
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

    % parse input arguments and user settings
    [Param, Opts] = loadParametersAndOptions(benchmark, Param, Opts);
                                            
    % compute safe terminal set
    termSet = computeSafeTerminalSet(Param, Opts);

    % compute safe initial set and corresponding input currection sets
    [set, inputSets] = computeSafeInitialSet(termSet,Param,Opts);
        
    % shift sets by equilibrium point
    set = set + Opts.xEq;
    termSet = termSet + Opts.xEq;

    % construct terminal region object
    K = Opts.XKX(Opts.nx+1:end, 1:Opts.nx);
    dt = Opts.cora.timeStep;

    T = termRegZonoLinSys(Opts.dynamics,set,Param,inputSets,K,Opts.xEq, ...
                                                    Opts.uEq,dt,termSet);