function Param = param_quadrotor2D()
% PARAM_QUADROTOR2D- parameters for the quadrotor2D benchmark
%
% Syntax:
%       Param = PARAM_QUADROTOR2D()
%
% Description:
%       Parameters for the 2D quadrotor benchmark. The parameters include
%       input constraints, disturbances as well as the parameters of the 
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
%       quadrotor2D
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
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------
    
    % set of admissible control inputs
    Param.U = interval([0;0],[0.2646;0.2646]);
    
    % set of uncertain disturbances
    Param.W = 0.1*interval(-ones(3,1),ones(3,1));
    
    % initial set of states
    width = [0.1; 0.1; 0.05; 0.1; 0.1; 0.05];
    Param.R0 = interval(-width,width);
    
    % final time
    Param.tFinal = 1;
    
    % desired final state
    Param.xf = [1; 1; 0; 0; 0; 0];

end