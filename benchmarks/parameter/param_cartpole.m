function Param = param_cartpole()
%PARAM_CARTPOLE - parameters for the cartpole benchmark
%
% Syntax:
%       Param = PARAM_CARTPOLE()
%
% Description:
%       Parameters for the cartpole benchmark. The parameters include state 
%       and input constraints as well as the set of uncertain disturbances.
%
% Output Arguments:
%
%       -Param:             a structure containing the following options
%           -.U:            set of admissible control inputs 
%                           (class: interval)
%           -.X:            set of admissible states (class: interval)
%           -.W:            set of uncertain disturbances (class: interval)
%
% See Also:
%       terminalRegion, termRegNonlinSysLinApproach, cartpole
%
% References:
%       [1] J. Theis "Sum-of-Squares Applications in Nonlinear Controller 
%           Synthesis", 2012, Report, University of California Berkeley
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2021 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------
    
    % set of admissible control inputs
    width = 10.0;
    Param.U = interval(-width, width);
    
    % set of uncertain disturbances
    width = [0.5;0.02];
    Param.W = interval(-width, width);

end