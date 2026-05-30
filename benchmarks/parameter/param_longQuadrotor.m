function Param = param_longQuadrotor()
%PARAM_LONGQUADROTOR - parameters for the longitudinal quadrotor benchmark 
%
% Syntax:
%       Param = PARAM_LONGQUADROTOR()
%
% Description:
%       Parameters for the longitudinal quadrotor benchmark. The parameters 
%       include state and input constraints as well as the set of uncertain 
%       disturbances.
%
% Output Arguments:
%
%       -Param:             a structure containing the following options
%           -.U:            set of admissible control inputs 
%                           (class: interval)
%           -.W:            set of uncertain disturbances 
%                           (class: interval)
%           -.X:            set of admissible states 
%                           (class: polytope)

%
% See Also:
%       longQuadrotor
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
    
    % set of admissible states
    lb = [-1.7; 0.3; -0.8; -1; -pi/12; -pi/2]; 
    ub = [1.7; 2.0; 0.8; 1.0; pi/12; pi/2];
    Param.X = polytope(interval(lb, ub));
    
    % set of admissible control inputs
    g = 9.81; K = 0.89/1.4;
    lb = [-1.5+g/K; -pi/12]; ub = [1.5+g/K; pi/12];
    Param.U = interval(lb, ub);
    
    % set of uncertain disturbances
    width = 0.1;
    Param.W = interval(-width, width);

end
