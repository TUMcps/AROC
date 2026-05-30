function Param = param_chainMSD(nMass)
%PARAM_CHAINMSD - parameters for the chain of mass-spring-damper systems benchmark
%
% Syntax:
%       Param = PARAM_CHAINMSD(nMass)
%
% Description:
%       Parameters for the coupled mass-spring-damper systems benchmark. 
%       The parameters include state and input constraints as well as the 
%       set of uncertain disturbances.
%
% Input Arguments:
%
%       -nMass:     number of coupled systems
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
%       chainMSD
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
    width = kron(ones(nMass,1), [6; 6]);
    Param.X = interval(-width, width);
    
    % set of admissible control inputs
    width = kron(ones(nMass,1), 14);
    Param.U = interval(-width, width);
    
    % set of uncertain disturbances
    width = kron(ones(nMass,1),0.15);
    Param.W = interval(-width, width);

end