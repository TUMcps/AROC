function sys = closedLoopSystemLin(A,B,D,c,K,u_ref)
% CLOSEDLOOPSYSTEMLIN - constructs a linear system for the closed-loop
%                        dynamics
%
% Syntax:
%       dx = CLOSEDLOOPSYSTEMLIN(A,B,D,c,K,u_ref)
%
% Description:
%       This function contructs a linear system object for the closed-loop
%       dynamics for the optimization based controller from the system
%       matrices of the uncontrolled system and the control law parameters.
%
% Input Arguments:
%
%       -A:     system matrix of the open-loop system
%       -B:     input matrix of the open-loop system
%       -D:     disturbance matrix of the open-loop system
%       -c:     constant offset vector of the open-loop system
%       -K:     feedback matrix for the control law u = u_ref + K(x-x_ref)
%       -u_ref: reference input for the control law u = u_ref + K(x-x_ref)
%
% Output Arguments:
%       -sys:   linear system object for the closed-loop dynamics 
%               (class: linearSys)
%
% See Also:
%       optimizationBasedControl
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
% Authors:      Ivan Hernandez, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    % construct system matrices
    A_ = [A + B*K,-B*K;
          zeros(size(A)),A];
    B_ = [D;zeros(size(D))];
    C_ = [c + B*u_ref;c + B*u_ref];
    
    % construct linear system object
    sys = linearSys('closedLoopSys',A_,B_,C_);
    
end