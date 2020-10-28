function Param = param_cart()
% PARAM_CART - parameters for the mass-spring-damper benchmark
%
% Syntax:
%       Param = PARAM_CART()
%
% Description:
%       Parameters for the cart benchmark. The parameters include input 
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
%           -.W:            set of uncertain disturbances (class: interval)
%           -.X:            set of state constraints (class: mptPolytope)
%
% See Also:
%       cart
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
    
    % initial set of states
    x0 = [0;0];
    width = [0.2; 0.2];
    Param.R0 = interval(x0-width,x0+width);
    
    % goal state and final time
    Param.xf = [2;0];
    Param.tFinal = 1;
    
    % set of admissible control inputs
    width = 14;
    Param.U = interval(-width,width);
    
    % set of uncertain disturbances
    width = [0.1;0.1];
    Param.W = interval(-width,width);

end