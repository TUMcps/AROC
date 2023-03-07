function [x,t,u] = simulateSys(sys,K,uc,xc,p,Opts)
% SIMULATESYS - simulation of the real system behaviour
%
% Syntax:
%       [x,t,u] = simulateSys(sys,K,uc,xc,p,Opts)
%
% Description:
%       This function simulates the behaviour of the real system. The value
%       for the disturbances are chosen at random.
%
% Input Arguments:  
%
%       -sys:       object containing the dynamics of the closed-loop 
%                   system (class: nonlinParamSys)
%       -K:         feedback matrix for the tracking controller 
%                   (dimension: [nu,nx])
%       -uc:        reference control input (dimension: [nu,1])
%       -xc:        reference point of center trajectory 
%                   (dimension: [nx,1])
%       -p:         initial point for the reachable set computation 
%                   (dimension: [nx,1])
%       -Opts:      a structure containing following options
%
%           -.nw:   number of system disturbances
%           -.W:    set of uncertain disturbances (class: interval)
%
% Output Arguments:
%
%       -x:     simulated trajectory     	
%       -t:     time points for the simulated trajectory
%       -u:     applied control inputs
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

    % initialization
    N = 20;
    x = []; t = []; u = [];
    options.Events = [];
    
    params.x0 = [p;xc(:,1)];
    params.tStart = 0;
    
    % get index of the current intermediate time step
    ind = floor(Opts.tComp / (Opts.dT/Opts.Ninter)) + 1;
   
    % loop over all remaining intermediate time steps
    for i = ind:Opts.Ninter    
        
        % update reference input and feedback matrix
        params.p = [reshape(K{i},[],1);uc(:,i)];
        params.tFinal = min(Opts.dT/Opts.Ninter, ...
                            i*(Opts.dT/Opts.Ninter) - Opts.tComp)/N;
        
        % loop over all disturbance changes
        for j = 1:N
        
            % update disturbances
            if isempty(Opts.V)
                params.u = randPoint(Opts.W,1,'extreme');
            else
                params.u = randPoint(cartProd(Opts.W,Opts.V),1,'extreme');
            end

            % simulate the system
            [tTemp,xTemp] = simulate(sys,params,options);

            % store the simulated trajectory
            x = [x;xTemp];
            if isempty(t)
               t = tTemp;
            else
                t = [t;tTemp+t(end)];
            end

            % update initial state
            params.x0 = x(end,:)';
            
            % compute control input
            uTemp = (uc(:,i)+K{i}'*(xTemp(:,1:Opts.nx)- ...
                                    xTemp(:,Opts.nx+1:end))')';
            u = [u;uTemp];
        end
    end
end