function [R,R0] = reachPoly(R0,pZ_U,Opts)
% REACHPOLY - computes the reachable set for a polynomial controller
%
% Syntax:
%       [R,R0] = REACHPOLY(R0,pZ_U,Opts)
%
% Description:
%       Computes the over-approximative closed-loop reachable set for a
%       polynomial control law
%
% Input Arguments:
%
%       -R0:        current initial set
%       -pZ_U:      polynomial zonotope representing the optimal controller
%       -Opts:      structure containing all options
%
% Output Arguments:
%
%       -R:         Closed-loop reachable set for controller pZCtrl_x 
%                   (class reachSet)
%       -R0:        Final time point reachable set (class polyZonotope)
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

    % initialization
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    R = [];

    % loop over all intermediate time steps
    for j = 1:Opts.Ninter

        % extract set of control inputs
        dim_u = (j-1)*Opts.nu+(1:Opts.nu);
        U = project(pZ_U,dim_u);

        % reachability parameter
        params.tStart = (j-1)*Opts.dt;
        params.tFinal = j*Opts.dt;
        params.R0 = stack(R0,U);  

        % reachability analysis
        R_ext = reach(Opts.sys,params,options);

        % store reachable set
        Rtmp = projectReachSet(R_ext,1:Opts.nx);
        R = add(R,Rtmp);
        Rtp = R_ext.timePoint.set{end};
        R0 = compact(project(Rtp,1:Opts.nx));
    end
end