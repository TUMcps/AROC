function dx = extendedDynamicsMeasErr(x,w,p,nx,nu,nw,forig,Ahan,Bhan)
% EXTENDEDDYNAMICSMEASERR - dynamic equtions for the controlled system
%
% Syntax:
%       dx = EXTENDEDDYNAMICSMEASERR(x,w,p,nx,nu,forig,Ahan,Bhan)
%
% Description:
%       This function implements the closed-loop dynamics for a system
%       controlled with the combined controller, where measurement errors
%       are explicitely considered.
%
% Input Arguments:  
%
%       -x:         extended system state (dimension: 5*nx,1])
%       -w:         disturbances (dimension: [nw+nx,1])
%       -p:         parameter vector containing control law parameter
%                   (dimension: [2*nu*nx+nx+2*nu,1])
%       -nu:        number of system states (scalar)
%       -nu:        number of system inputs (scalar)
%       -nw:        number of disturbances (scalar)
%       -forig:     function handle to the dynamic function of the original
%                   open-loop system
%       -Ahan:      function handle to the system matrix of the linearized
%                   system
%       -Bhan:      function handle to the input matrix of the linearized
%                   system
%
% Output Arguements:
%
%       -dx:        value of the dynamic equtions (dimension: [4*nx,1])
%
% See Also:
%       combinedControl
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
% Authors:      Victor Gassmann, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % disassemble extendet state x = [x_orig; x_ref; dx_ff; beta; v_init]
    x_orig = x(1:nx);
    x_ref = x(nx+1:2*nx);
    dxff = x(2*nx+1:3*nx);
    beta = x(3*nx+1:4*nx);
    v_init = x(4*nx+1:5*nx);
    
    % disassemble disturbance vector into disturbance and measurement error
    v = w(nw+1:nw+nx);
    w_ = w(1:nw);

    % get constant parameter
    K = reshape(p(1:nu*nx),[nu,nx]);    
    xlin = p(nu*nx+1:nu*nx+nx);
    ulin = p(nu*nx+nx+1:nu*nx+nx+nu);
    uc = p(nu*nx+nx+nu+1:nu*nx+nx+2*nu);
    GuAlpha = reshape(p(nu*nx+nx+2*nu+1:2*nu*nx+nx+2*nu),[nu,nx]);

    % get matrices A,B for linearized system
    A = Ahan(xlin,ulin,zeros(size(w_)));
    B = Bhan(xlin,ulin,zeros(size(w_)));

    % construct control law
    uff = uc + GuAlpha*(beta + v_init);
    xff = x_ref + dxff;
    u_ctrl = uff + K*(x_orig + v - xff);

    % dynamics for original state state x_orig
    dx(1:nx,1) = forig(x_orig,u_ctrl,w_);

    % dynamics for reference trajectory x_ref
    dx(nx+1:2*nx,1) = forig(x_ref,ulin,zeros(size(w_)));

    % dynamics for diff. of feedforward and ref. traj dx_ff = x_ff - x_ref
    dx(2*nx+1:3*nx,1) = A*dxff + B*(uff - ulin);

    % dynamics for the factors beta 
    dx(3*nx+1:4*nx,1) = zeros(nx,1);
    
    % dynamics for initial measurement error v_init
    dx(4*nx+1:5*nx,1) = zeros(nx,1);
    
end