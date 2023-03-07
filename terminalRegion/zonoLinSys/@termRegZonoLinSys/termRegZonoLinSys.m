classdef termRegZonoLinSys < terminalRegion
% TERMREGZONOLINSYS - terminal region object computed with the linear 
%                     system approach using zonotopes
%
% Syntax:
%       obj = TERMREGZONOLINSYS(dyn,set,Param,inpSets,K,xEq,uEq,dt,termSet)
%
% Description:
%       This class represents a terminal region computed with the
%       zonotope approach for linear systems in [1].
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -set:       set representing the terminal region (class: zonotope) 
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
%       -inpSet:    cell-array storing the correction zonotopes
%       -K:         feedback matrix for the terminal controller
%       -xEq:       equilibrium point of the system
%       -uEq:       control inputs for the equilibrium point
%       -dt:        time step size for the sampled-data controller
%       -termSet:   terminal set from first iteration (class: zonotope)
%
% Output Arguments:
%
%       -obj:      resulting object of class termRegLinSysApproach
%
% See Also:
%       terminalRegion, computeTermZonoRegLinSys
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

properties (SetAccess = private, GetAccess = public)
    inputSets = [];     % control input sets
    K = [];             % feedback matrices
    xEq = [];           % equilibrium point
    uEq = [];           % control input for the equilibrium point
    dt = [];            % time step size
    termSet = [];       % terminal set after the first phase
end

methods
    % class constructor
    function obj = termRegZonoLinSys(dyn,set,Param, ...
                                              inpSets,K,xEq,uEq,dt,termSet)

        % call superclass constructor
        obj = obj@terminalRegion(dyn,set,Param);

        % store object properties
        obj.inputSets = inpSets;
        obj.K = K;
        obj.xEq = xEq;
        obj.uEq = uEq;
        obj.dt = dt;
        obj.termSet = termSet;
    end
end
end