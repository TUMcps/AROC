function Param = param_artificialSystem()
% PARAM_ARTIFICIALSYSTEM - parameters for the artificial system benchmark
%
% Syntax:
%       Param = PARAM_ARTIFICIALSYSTEM()
%
% Description:
%       Parameters for the artificial system benchmark. The parameters 
%       include input constraints, disturbances as well as the parameters 
%       of the motion primitive like initial set, goal state, etc..
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
%       artificial system, param_car
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
    Param.U = interval(-2,2);
    
    % set of uncertain disturbances
    Param.W = interval(-0.1,0.1);
    
    % initial set of states
    x0 = [0.6;-0.6];
    width = [0.1; 0.05];
    Param.R0 = interval(x0-width,x0+width);
    
    % goal state
    Param.xf = [0;0];

end