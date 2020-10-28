function dx = closedLoopSystemNonlin(x,w,p,f,nx,nu,nw)
% CLOSEDLOOPSYSTEMNONLIN - dynamic function for the closed-loop dynamics
%
% Syntax:
%       dx = CLOSEDLOOPSYSTEMNONLIN(x,w,p,f,nx,nu,nw)
%
% Description:
%       The function implements the dynamics for the closed-loop system for
%       the optimization based controller for a nonlinear system.
%
% Input Arguments:
%
%       -x:     state vector
%       -w:     disturbance vector
%       -p:     parameter vector
%       -f:     function handle for the open-loop dynamics
%       -nx:    number of states
%       -nu:    number of inputs
%       -nw:    number of disturbances
%
% Output Arguments:
%       -dx:    time derivative for the closed-loop dynamics
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

    % extract the reference input and the feedback matrix from the
    % parameter vector
    u_ref = p(1:nu);
    K = reshape(p(nu+1:end),[nu,nx]);
    
    % compute the control law
    u = u_ref + K * (x(1:nx)-x(nx+1:end));
    
    % construct the derivative of the extendet system
    dx1 = f(x(1:nx),u,w);
    if isa(x,'sym')
        dx2 = f(x(nx+1:end),sym(u_ref),sym(zeros(nw,1)));
    else
        dx2 = f(x(nx+1:end),u_ref,zeros(nw,1));
    end
    
    dx = [dx1;dx2];
end