function [Rfin,Rcont] = compReachSet(sys,K,uc,xc,p,Opts)
% COMPREACHSET - reachable set computation for the closed-loop system
%
% Syntax:
%       [R,Rcont] = COMPREACHSET(sys,K,uc,xc,p,Opts)
%
% Description:
%       This function computes the reachable set for the closed loop system 
%       starting from teh point p.
%
% Input Arguments:  
%
%       -sys:   object containing the dynamics of the closed-loop system 
%               (class: nonlinParamSys)
%       -K:     feedback matrix for the tracking controller 
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
    
    if isempty(Opts.V)
       params.R0 = zonotope([p;xc(:,1)]);
    else
       params.R0 = cartProd(Opts.V,zonotope(zeros(Opts.nx,1)))+[p;xc(:,1)]; 
    end
    if strcmp(options.alg,'poly')
        params.R0 = polyZonotope(params.R0);
    end
    
    tFin = Opts.tStart + Opts.tComp;
    params.tStart = Opts.tStart;
    
    Rcont = []; cnt = 1;

    % loop over all intermediate time steps until allocated time is reached
    while true
        
        % update reachability settings
        params.paramInt = [reshape(K{cnt},[],1);uc(:,cnt)];
        params.tFinal = min(tFin,params.tStart + Opts.dT/Opts.Ninter);
        options.timeStep = (params.tFinal-params.tStart)/Opts.reachSteps;
        cnt = cnt + 1;
        
        % reachability analysis 
        Rtemp = reachNonlinear(sys,params,options);
        
        % update initial set and store reachable set
        Rcont = add(Rcont,Rtemp);
        params.R0 = Rtemp.timePoint.set{end};
        params.tStart = params.tStart + Opts.dT/Opts.Ninter;
        
        if abs(tFin - params.tFinal) <= eps
            break;
        end
    end
    
    % final reachable set
    Rfin = project(Rtemp.timePoint.set{end},1:Opts.nx);
end