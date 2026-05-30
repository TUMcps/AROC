function Param = param_robotArm()
% PARAM_ROBOTARM - parameters for the robot arm benchmark
%
% Syntax:
%       Param = PARAM_ROBOTARM()
%
% Description:
%       Parameters for the robot arm benchmark. The parameters include
%       input constraints, disturbances as well as the parameters of the
%       motion primitive like initial set, goal state, etc..
%
% Output Arguments:
%
%       -Param:             a structure containing following options
%
%           -.R0:           initial set of states 
%                           (class: interval)
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal state should be
%                           reached
%           -.U:            set of admissible control inputs 
%                           (class: interval)
%           -.W:            set of uncertain disturbances 
%                           (class: interval)
%           -.X:            set of admissible states 
%                           (class: polytope)
%
% See Also:
%       robotArm
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
    x0 = [pi/6;0;0;0];
    width = [0.05; 0.05; 0.05; 0.05];
    Param.R0 = interval(x0-width,x0+width);
    
    % goal state and final time
    Param.xf = [0.75*pi;0.4482;0;0];
    Param.tFinal = 1;

    % set of admissible states
    tmp = polytope(interval([0;-pi], [pi;pi]));
    A = zeros(4,4);
    A(:, [1 2]) = tmp.A;

    Param.X = polytope(A, tmp.b);
    
    % set of admissible control inputs
    width = [3;1];
    Param.U = interval(-width,width);
    
    % set of uncertain disturbances
    width = [0.01;0.01;0.01;0.01];
    Param.W = interval(-width,width);

end