function Param = param_truck()
% PARAM_TRUCK - parameters for the truck benchmark
%
% Syntax:
%       Param = PARAM_TRUCK()
%
% Description:
%       Parameters for the truck benchmark. The parameters include input
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
%       truck
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
    
    % set of admissible control inputs
    width = [1;9.81];
    Param.U = interval(-width,width);
    
    % set of uncertain disturbances
    width = [0.02;0.5];
    Param.W = interval(-width,width);
    
    % initial set of states
    x0 = [0;0;0;15;0;0];
    width = [0.02; 0.02; 0.02; 0.2; 0.2; 0.2];
    Param.R0 = interval(x0-width,x0+width);

    % state constraints
    tmp = mptPolytope(interval([-0.9;-pi/2],[0.9;pi/2]));

    A = zeros(4,6);
    A(:, [1, 3]) = tmp.P.A;
    
    Param.X = mptPolytope(A,tmp.P.b);
    
    % final time
    Param.tFinal = 3;

    % final state
    Param.xf = [0; 0; 0; 15; 45; 3.68];

end