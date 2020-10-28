function [Rfin,Rcont] = compReachSet(sys,k,uc,xc,p,Opts)
% COMPREACHSET - reachable set computation for the closed-loop system
%
% Syntax:
%       [R,Rcont] = COMPREACHSET(sys,k,uc,xc,p,Opts)
%
% Description:
%       This function computes the reachable set for the closed loop system 
%       starting from teh point p.
%
% Input Arguments:  
%
%       -sys:   object containing the dynamics of the closed-loop system 
%               (class: nonlinParamSys)
%       -k:     feedback matrix for the tracking controller 
%               (dimension: [nu,nx])
%       -uc:    reference control input (dimension: [nu,1])
%       -xc:    reference point of center trajectory (dimension: [nx,1])
%       -p:     initial point for the reachable set computation 
%               (dimension: [nx,1])
%       -Opts:  a structure containing following options
%
%           -.nx:               number of system states
%           -.tComp:            time allocated to perform the computations
%                               for the optimizations 
%                               (0 < Opts.tComp < Opts.dT).
%           -.ReachOptsTrack:   reachability options for the CORA toolbox
%
% Output Arguments:
%
%       -Rfin:      final reachable set
%       -Rcont:     cell-array containing the time interval reachable sets
%
% See Also:
%       reachsetMPC
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

    % reachability settings
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    
    params.paramInt = [reshape(k,[],1);uc];
    params.R0 = zonotope([p;xc]);
    params.tFinal = Opts.tStart + Opts.tComp;
    
    if Opts.tStart ~= 0
        params.tStart = Opts.tStart;
    end
    
    options.timeStep = Opts.tComp/Opts.reachSteps;

    % reachability analysis
    Rcont = reachNonlinear(sys,params,options);
    
    % final reachable set
    Rfin = project(Rcont.timePoint.set{end},1:Opts.nx);
end