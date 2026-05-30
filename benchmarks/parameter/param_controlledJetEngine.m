function Param = param_controlledJetEngine()
%PARAM_CONTROLLEDJETENGINE - parameters for the controlled jet engine
%       benchmark
%
% Syntax:
%       Param = PARAM_CONTROLLEDJETENGINE()
%
% Description:
%       Parameters for the controlled jet engine benchmark. The parameters 
%       include the set of admissibel states, the input constraints, and 
%       the set of disturbances.
%
% Output Arguments:
%
%       -Param:             a structure containing following options
%           -.U:            set of admissible control inputs 
%                           (class: interval)
%           -.W:            set of uncertain disturbances 
%                           (class: interval)
%           -.X:            set of admissible states
%                           (class: polytope)
%
% See Also:
%       controlledJetEngine
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
% Authors:      Niklas Kochdumper, Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% set of admissible states
width = [0.2; 0.2];  
Param.X = polytope(interval(-width, width));

% set of admissible control inputs
Param.U = interval(-0.35, 0.35);

% set of uncertain disturbances
Param.W = interval(-0.025, 0.025);

end
