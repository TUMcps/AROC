function [u,x] = optimalControlFmincon(system,xf,x0,h,Q,R,steps,horizon,Opts)
% OPTIMALCONTROLFMINCON - Solve optimal control problem with FMINCON 
%
% Syntax:
%       [u,x] = OPTIMALCONTROLFMINCON(system,xf,x0,h,Q,R,steps,horizon,Opts)
%
% Description:
%       This function solves an optimal control problem using a multiple
%       shooting algorithm and FMINCON
%
% Input Arguments:
%
%       -system:        object containing the system dynamics (class:
%                       nonlinearSys) 
%       -xf:            desired final state at the end of the control
%                       process (dimension: [nx, 1])
%       -x0:            initial state (dimension: [nx, 1])
%       -h:             length of one timestep of the corner trajectories
%                       during which the control input is constant 
%       -Q:             weighting matrix for the final state of the optimal
%                       control problem
%                       (dimension: [nx,nx])
%       -R:             weighting matrix for the input term of the optimal
%                       control problem
%                       (dimension: [nu,nu])
%       -steps:         number of intermediate timesteps of the corner
%                       trajectories during one timestep of the center
%                       trajectory
%       -lenHorizon:    length of the optimization horizon in center
%                       trajectory time steps
%       -Opts:          structure containing user defined options for the 
%                       algorithm  
%
% Output Arguments:
%
%       -u:             optimal control input
%                       (dimension: [nu,steps*horizon])
%       -x:             resulting state trajectories
%                       (dimension: [nx,steps*horizon])
%
% See Also:
%       convexInterpolationControl, writeFminconFiles
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

    % solve optimal control problem
    inter = 5;
    N = steps*horizon;
    
    text = sprintf(['cost = @(x) fmincon_%i_%i_cost', ...
                                    '(x,x0,xf,Q,R,h);'],steps,horizon);
    eval(text);
    
    text = sprintf(['con = @(x) fmincon_%i_%i_con', ...
                                    '(x,x0,xf,Q,R,h);'],steps,horizon);
    eval(text);
    
    options = optimoptions('fmincon','Algorithm', ...
                            'interior-point','Display','off', ...
                            'MaxFunctionEvaluations',1e6); 
                        
    nOpt = Opts.nx*(N*inter+1) + Opts.nu*N;
    xInit = zeros(nOpt,1);
    
    lb = -inf*ones(nOpt,1);
    ub = inf*ones(nOpt,1);
    
    lb(Opts.nx*(N*inter+1)+1:end) = repmat(Opts.uMin,[N,1]);
    ub(Opts.nx*(N*inter+1)+1:end) = repmat(Opts.uMax,[N,1]);
    
    
    sol = fmincon(cost,xInit,[],[],[],[],lb,ub,con,options);

    x_ = sol(1:Opts.nx*(N*inter+1));
    x_ = reshape(x_,[Opts.nx,N*inter+1]);
    u = sol(numel(x_)+1:end);
    u = reshape(u,[Opts.nu,N]);

    
    % simulate trajectory to obtain more accurate result
    x = zeros(Opts.nx,N+1);
    x(:,1) = x0;
    
    for k=1:N
        [~,x_temp]=ode45(@(t,x)system(x,u(:,k),zeros(Opts.nw,1)), ...
                                                            [0 h],x(:,k)');
        x(:,k+1) = x_temp(end,:)';
    end
end