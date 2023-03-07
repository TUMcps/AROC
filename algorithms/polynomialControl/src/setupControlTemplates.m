function [Uscalar,U] = setupControlTemplates(Opts)
% SETUPCONTROLTEMPLATES - initialize templates for polynomial controllers
%
% Syntax:
%       [Uscalar,U] = SETUPCONTROLTEMPLATES(Opts)
%
% Description:
%       Initialize the polynomial zonotopes that represent the polynomial
%       control laws.
%
% Input Arguments:
%
%       -Opts:  structure containing all options
%
% Output Arguments:
%
%       -Uscalar:   template for the control law for a scalar control input
%       -U:         template for the control law for multiple control
%                   inputs and multiple time steps
%
% See Also:
%       polynomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % template for a single scalar control input
    G = ones(1,Opts.np_s);
    expMat = [Opts.eMCtrl;eye(Opts.np_s)];
    id = [Opts.idw;Opts.idp_s];

    Uscalar = polyZonotope(0,G,[],expMat,id);
    
    % template for all control inputs
    Gscalar = repelem(eye(Opts.nu),1,Opts.np_s);
    Gtmp = repmat({Gscalar},Opts.Ninter,1);
    G = blkdiag(Gtmp{:});

    c = zeros(Opts.nu*Opts.Ninter,1);
    Grest = zeros(Opts.nu*Opts.Ninter,0);
    expMat = [repmat(Opts.eMCtrl,1,Opts.nu*Opts.Ninter);eye(Opts.Np)];
    id = [Opts.idw;Opts.idp];

    U = polyZonotope(c,G,Grest,expMat,id);
                         
    % unit hypercube
    c = zeros(Opts.nx,1);
    G = [eye(Opts.nx),eye(Opts.nx)];
    Grest = zeros(Opts.nx,0);
    expMat = eye(2*Opts.nx);
    id = [Opts.idw;Opts.idc];

    cube = polyZonotope(c,G,Grest,expMat,id);
    
    % insert unit hypercube into templates to represent the parameter set
    Uscalar = subs(Uscalar,cube,Opts.idw);
    U = subs(U,cube,Opts.idw);
end