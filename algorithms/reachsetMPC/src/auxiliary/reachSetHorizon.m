function [param,res] = reachSetHorizon(sys,R0,param,Opts)
% REACHSETHORIZON - reachable set computation for the closed-loop system
%                   for the whole prediction horizon
%
% Syntax:
%       [param,res] = REACHSETHORIZON(sys,R0,param,Opts)
%
% Description:
%       This function computes the reachable set for the closed loop system 
%       for the whole prediction horizon
%
% Input Arguments:  
%
%       -sys:   object containing the dynamics of the closed-loop system 
%               (class: nonlinParamSys)
%       -R0:    initial set for reachablity analysis (class: zonotope)
%       -param: a structure containing following options
%
%           -.xc:       reference trajectory (dimension: [nx,N+1])   
%           -.uc:       reference trajectory control inputs
%                       (dimension: [nu,N])
%           -.K:        cell-array containing the computed feedback
%                       matrices for the tracking controller
%           -.R:        cell-array containing the time point reachable sets
%           -.Rcont:    cell-array containing the time interval reachable
%                       sets
%           -.J:        summed distance of the reference trajectory points
%                       from the terminal region
%           -.L:        value of the objective function (=cost) of the 
%                       reference trajectory
%           -.Jset:     summed distance of the reachable sets from the
%                       terminal region
%
%       -Opts:  a structure containing following options
%
%           -.nx:               number of system states
%           -.dT:               time step. Prediction horizon: 
%                               Opts.N * Opts.dT 
%           -.termReg:          terminal region around the steady state xf
%                               (class: mptPolytope)
%           -.fbContr:          struct with name and parameter of the
%                               applied tracking controller
%           -.ReachOptsTrack:   reachability options for the CORA toolbox
%
% Output Arguments:
%
%       -param:     updated struct of control law parameters
%       -res:       flag that specifies if final reachable set is fully 
%                   contained in the terminal region (0 or 1)
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

    % initialize variables
    N = size(param.uc,2);
    res = 0;
    tStart = Opts.tStart;
    
    param.Rcont = [];
    param.R = cell(N,1);
    param.K = cell(N,1);

    % loop over all center trajectory time steps
    for i = 1:N
        
        % compute feedback matrix for tracking controller
        K = lqrTrack(param.xc(:,i),param.xc(:,i+1),param.uc(:,i),Opts);
                               
        % compute reachable set
        ReachOpts = Opts.ReachOpts;
        ReachParams = Opts.ReachParams;
        
        ReachParams.paramInt = [reshape(K,[],1);param.uc(:,i)];
        ReachParams.R0 = cartProd(R0,zonotope(param.xc(:,i)));
        ReachParams.tFinal = tStart + Opts.dT;
        
        if tStart ~= 0
            ReachParams.tStart = tStart;         
        end
        tStart = ReachParams.tFinal;

        Rtemp = reachNonlinear(sys,ReachParams,ReachOpts);
        
        % store variables
        param.K{i} = K;
        param.Rcont = add(param.Rcont,Rtemp);
        param.R{i} = project(Rtemp.timePoint.set{end},1:Opts.nx);
        
        % update initial set
        R0 = param.R{i};
        
        % check if input constraints are satisfied
        resInp = checkInputConstraints(Rtemp.timeInterval.set,K, ...
                                       param.uc(:,i),Opts);
        
        if ~resInp
           break; 
        end
        
        % check if the current set is inside the terminal region
        distance = distPhi(Opts.termReg.A,Opts.termReg.b,R0);

        if distance == 0
           res = 1;
           param.K = param.K(1:i);
           param.Rcont = param.Rcont(1:i);
           param.R = param.R(1:i);
           break; 
        end     
    end
    
    % compute distance costs for the reachable sets
    if res
        param.Jset = costSetDistance(param.R,Opts);
    end
end