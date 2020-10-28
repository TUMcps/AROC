function [x,t,u] = simulateSys(sys,k,uc,xc,p,tFinal,Opts)
% SIMULATESYS - simulation of the real system behaviour
%
% Syntax:
%       [x,t,u] = simulateSys(sys,k,uc,xc,p,tFinal,Opts)
%
% Description:
%       This function simulates the behaviour of the real system. The value
%       for the disturbances are chosen at random
%
% Input Arguments:  
%
%       -sys:       object containing the dynamics of the closed-loop 
%                   system (class: nonlinParamSys)
%       -k:         feedback matrix for the tracking controller 
%                   (dimension: [nu,nx])
%       -uc:        reference control input (dimension: [nu,1])
%       -xc:        reference point of center trajectory 
%                   (dimension: [nx,1])
%       -p:         initial point for the reachable set computation 
%                   (dimension: [nx,1])
%       -tFinal:    time horizon for the simulation
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
    options.Events = [];
    x = []; t = [];
    
    params.x0 = [p;xc];
    params.p = [reshape(k,[],1);uc];
    params.tStart = 0;
    params.tFinal = tFinal/N;
    
    % loop over all time steps
    for i = 1:N
        
        % update disturbances
        params.u = randPointExtreme(Opts.W);
       
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
    end

    % compute control input
    u = (uc + k'*(x(:,1:Opts.nx)-x(:,Opts.nx+1:end))')';

end