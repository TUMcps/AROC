function f = closedLoopTrackingMeasErr(x,w,p,dyn,nu)
% CLOSEDLOOPTRACKINGMEASERR - dynamic equtions for the controlled system 
%                             (tracking controller) 
%
% Syntax:
%       f = CLOSEDLOOPTRACKINGMEASERR(x,w,p,dyn,nu)
%
% Description:
%       This function contains the dynamic equations for the system
%       controlled with the tracking controller u = u_ref + K (x_ - x_ref),
%       where x_ is the measured state including a potential uncertain 
%       measurement error.
%
% Input Arguments:  
%
%       -x:         extended system state [x;x_ref] (dimension: [2*nx,1])
%       -w:         disturbances (dimension: [nw,1])
%       -p:         parameter vector containing the reference control
%                   input and the entries of the feedback matrix
%                   (dimension: [nu*(nx+1),1])
%       -dyn:       function handle specifying the open-loop dynamics
%       -nu:        number of system inputs (scalar)
%
% Output Arguements:
%
%       -f:   value of the dynamic equtions (dimension: [2*nx,1])
%
% See Also:
%       reachsetMPC
%
% References:
%       * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
%              for Disturbed Nonlinear Systems
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

    % Extract parameter
    nx = length(x)/2;
    
    index  = 1;
    for i = 1:nu
       K(:,i) = p(index:index+nx-1);
       index = index + nx;
    end
    
    uref = p(index:end);
    
    X = x(1:nx,1);
    xref = x(nx+1:end,1);
    W = w(1:length(w)-nx);
    v = w(length(w)-nx+1:end);

    % Control law (see Eq. (5) in [1])
    u = uref + transpose(K)*(X + v - xref);
    
    % Controlled system dynamics
    f = [dyn(X,u,W);dyn(xref,uref,zeros(length(W),1))];

end