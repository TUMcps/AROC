function Param = param_stirredTankReactor_traj1()
% PARAM_STIRREDTANKREACTOR_TRAJ1 - parameters for the stirred-tank reactor
%                                  benchmark
%
% Syntax:
%       Param = PARAM_STIRREDTANKREACTOR_TRAJ1()
%
% Description:
%       Parameters for the stirred-tank reactor benchmark. The parameters 
%       include input constraints, disturbances as well as the parameters 
%       of the motion primitive like initial set, goal state, etc..
%
% Output Arguments:
%
%       -Param:             a structure containing following options
%
%           -.R0:           initial set (class: interval)
%           -.xf:           goal state
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval)
%
% See Also:
%       stirredTankReactor
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
    
    % load disturances, input constraints, and final set
    Param = param_stirredTankReactor();
    
    % initial set
    x0 = [-0.15;-45];
    Param.R0 = interval(x0);

end