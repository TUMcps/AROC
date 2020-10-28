function [Rfinal,reachSet] = reachExactController(dynamic,R0,param,Opts)
% REACHEXACTCONTROLLER - reachabilty analysis for the exact control law 
%                        implementation
%
% Syntax:
%       [Rfinal,reachSet] = REACHEXACTCONTROLLER(dynamic,R0,param,Opts)
%
% Description:
%       Computes the reachable sets at the end of one timestep of the
%       center trajectory as well as all reachable sets during this
%       timestep
%
% Input Arguments:
%
%       -dynamic:       object containing the system dynamics (class:
%                       nonlinParamSys)  
%       -R0:            extended initial zonotope, which contains the state  
%                       as well as the auxiliary states (dimension: 2*nx,
%                       class zonotope or polyZonotope)
%       -param:         parameter vector. Containts the optimal inputs for 
%                       the vertices of the parallelotope, which are needed 
%                       for the online implementation of the control law 
%                       (dimension: [nu*(2^nx),1])
%       -Opts:          structure containing user defined options for the 
%                       algorithm   
%
% Output Arguments:
%
%       -Rfinal:        final zonotope at the end of reachability analysis
%       -reachSet:      cell array containing the reachable sets of all
%                       intermediate time steps
%
% See Also:
%       convexInterpolationControl, computeExactController
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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

    reachSet = [];
    tStart = Opts.tStart;

    % get reachability options
    options = Opts.ReachOpts;
    params = Opts.ReachParams;

    % reachable set is computed for one time step of the center trajectory, 
    % which consists of Ninter intermediate steps
    for l = 1:Opts.Ninter   

        % set new input values by changing the center of the disturbance
        % zonotope
        params.paramInt = param(:,l);
            
        % update initial set and time for reachability analysis
        params.R0 = R0;
        if tStart ~= 0
            params.tStart = tStart;
            params.tFinal = params.tStart + Opts.hinter;
        else
            params.tFinal = Opts.hinter;
        end
        
        % compute reachable set
        Rtemp = reach(dynamic,params,options);

        % obtain reachable set for time intervalls and final time point
        Rfinal = Rtemp.timePoint.set{end};
        R0 = Rfinal;
        reachSet = add(reachSet,Rtemp);
        tStart = params.tFinal;
    end
end