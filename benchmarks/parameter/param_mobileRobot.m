function Param = param_mobileRobot()
% PARAM_MOBILEROBOT - parameters for the mobile robot benchmark
%
% Syntax:
%       Param = PARAM_MOBILEROBOT()
%
% Description:
%       Parameters for the mobile robot benchmark. The parameters include 
%       input constraints, disturbances as well as the parameters of the 
%       motion primitive like initial set, goal state, etc..
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
%       mobileRobot, param_car
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
    width = [0.5;0.5];
    Param.U = interval(-width,width);
    
    % set of uncertain disturbances
    width = [0.001;0.001];
    Param.W = interval(-width,width);
    
    % initial set of states
    x0 = [0;0;0;3;3];
    width = [0.05; 0.05; 0.05; 0.05; 0.05];
    Param.R0 = interval(x0-width,x0+width);
    
    % final time
    Param.tFinal = 10;
    
    % goal state
    Param.xf = [0.5;0;0;50;50];

end