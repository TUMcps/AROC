function Param = param_ship()
% PARAM_SHIP - parameters for the ship benchmark
%
% Syntax:
%       Param = PARAM_SHIP()
%
% Description:
%       Parameters for the ship benchmark. The parameters include input
%       constraints, disturbances as well as the parameters of the motion
%       primitive like initial set, goal state, etc..
%
% Output Arguments:
%
%       -Param:             a structure containing following options
%
%           -.R0:           initial set of states (class: interval)
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal state should be
%                           reached
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval 
%                           or zonotope)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
% See Also:
%       ship
%
% References:
%       * *[1] Krasowski et al. (2022)*, CommonOcean: Vessel Models
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------
    
    % set of admissible control inputs (see Table 1 in [1])
    width = [5894896.77;5894896.77;1350409.48];
    Param.U = interval(-width,width);
    
    % set of uncertain disturbances
    width = [1000;1000;1000];
    Param.W = interval(-width,width);
    
    % initial set of states
    x0 = [0;0;0;5;0;0];
    width = [1;1;0.01;0.2;0.2;0.001];
    Param.R0 = interval(x0-width,x0+width);
    
    % final time
    Param.tFinal = 30;

    % goal state
    Param.xf = [150;0;0;5;0;0];
end