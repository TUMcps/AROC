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
    N = length(param.uc);
    res = 0;
    tStart = Opts.tStart;
    
    param.Rcont = cell(N,1);
    param.R = cell(N,1);
    param.K = cell(N,1);

    % loop over all center trajectory time steps
    for i = 1:N
        
        param.K{i} = cell(Opts.Ninter,1);
        param.Rcont{i} = [];
        
        % loop over all intermediate time steps
        for j = 1:Opts.Ninter
        
            xc = param.xc{i}; uc = param.uc{i};
            
            % compute feedback matrix for tracking controller
            K = lqrTrack(xc(:,j),xc(:,j+1),uc(:,j),Opts);

            % compute reachable set
            ReachOpts = Opts.ReachOpts;
            ReachParams = Opts.ReachParams;

            ReachParams.paramInt = [reshape(K,[],1);uc(:,j)];
            ReachParams.R0 = cartProd(R0,zonotope(xc(:,j)));
            ReachParams.tFinal = tStart + Opts.dT/Opts.Ninter;

            if tStart ~= 0
                ReachParams.tStart = tStart;         
            end
            tStart = ReachParams.tFinal;

            Rtemp = reachNonlinear(sys,ReachParams,ReachOpts);

            % store variables
            param.K{i}{j} = K;
            param.Rcont{i} = add(param.Rcont{i},Rtemp);
            Rfin = project(Rtemp.timePoint.set{end},1:Opts.nx);

            % update initial set
            R0 = Rfin;

            % check if input constraints are satisfied
            resInp = checkInputConstraints(Rtemp.timeInterval.set,K, ...
                                           uc(:,j),Opts);

            % check if state constraints are satisfied
            resState = 1;

            if ~isempty(Opts.X)
                for k = 1:length(Rtemp.timeInterval.set)
                    I = interval(Opts.X.P.A * ...
                            project(Rtemp.timeInterval.set{k},1:Opts.nx));
                    if ~all(supremum(I) <= Opts.X.P.b)
                        resState = 0;
                        break;
                    end
                end
            end

            if ~resInp || ~resState
               return; 
            end
        end
        
        param.R{i} = R0;
        
        % check if the current set is inside the terminal region
        if isempty(Opts.V)
            Rtemp = R0 + (-Opts.xf);
        else
            Rtemp = R0 + Opts.V + (-Opts.xf);
        end
        
        distance = distPhi(Opts.termReg.A,Opts.termReg.b,Rtemp);

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