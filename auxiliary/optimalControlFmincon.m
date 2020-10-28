function [u,x] = optimalControlFmincon(system,xf,x0,h,Q,R,steps,u0,maxFunEvals,lenHorizon,Opts)
% OPTIMALCONTROLFMINCON - Solve optimal control problem with FMINCON 
%
% Syntax:
%       [u,x] = OPTIMALCONTROLFMINCON(system,xf,x0,h,Q,R,steps, ...
%                                     u0,maxFunEvals,lenHorizon,Opts)
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
%       -vert:          vertices of the initial parallelotope.
%                       (dimension: [nu,2^nx])
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
%       -u0:            input value for computing the initial point for 
%                       fmincon by integrating with Runge-Kutta
%                       (scalar value)
%       -maxFunEvals:   Maximum number of function evaluations for fmincon
%       -lenHorizon:    length of the optimization horizon in center
%                       trajectory time steps
%       -Opts:          structure containing user defined options for the 
%                       algorithm  
%
% Output Arguments:
%
%       -u:             optimal control input
%                       (dimension: [nu,steps*lenHorizon])
%       -x:             resulting state trajectories
%                       (dimension: [nx,steps*lenHorizon])
%
% See Also:
%       convexInterpolationControl, optimalControlFmincon
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

    % extract dimensions
    nx = Opts.nx;                           % number of state dimensions
    nu = Opts.nu;                           % number of input dimensions
    N = steps*lenHorizon;                   % number of overall time steps
    
    % default settings for extendet optimization horizon
    extHorizon.active = false;
    extHorizon.decay = 'uniform';
    extHorizon.horizon = 'all';
    
    if ~isfield(Opts,'extHorizon')
       Opts.extHorizon = extHorizon;
    end
    
    % enlarged input cost matrix for easier computation
    R = h*kron(eye(N),R);

    % initialization of initial solution for optimal control problem by
    % integrating the states for very small control inputs
    u0 = u0*ones(nu,N);
    xc0=zeros(nx,N);
    xc0=[x0,xc0];
    for k=1:N
        [~,x_temp]=ode45(@(t,x)system(x,u0(:,k),zeros(Opts.nw,1)),[0 h],xc0(:,k)');
        xc0(:,k+1) = x_temp(end,:)';
    end
    
    % combine state and input trajectory as initial solution for optimal
    % control problem
    xc0=reshape(xc0(:,2:end),[nx*N,1]);
    u0=reshape(u0,[nu*N,1]);
    xc0=[xc0;u0];

    % lower and upper bounds for states and inputs
    u_lb=repmat(Opts.uMin,[N,1]);
    u_ub=repmat(Opts.uMax,[N,1]);
    x_lb=-inf*ones(nx*N,1);    % no state constraints
    x_ub=inf*ones(nx*N,1);
    
    % compute decaying weigthing matrix for extended xVector
    decay = decayFunctions(lenHorizon,Opts);
    Qext = zeros(N);
    index = (steps-1)*nx+1;
    for i = 1:lenHorizon
       Qext(index:index+nx-1,index:index+nx-1) = decay(i) * Q; 
       index = index+steps*nx;
    end
    
    % concatanate desired final states to a vector
    xf_ = zeros(N*nx,1);
    index = (steps-1)*nx+1;
    for i = 1:lenHorizon
       xf_(index:index+nx-1) = xf(:,i); 
       index = index+steps*nx;
    end

    % solve optimal control problem using a multiple shooting algorithm and
    % fmincon for optimization
    system_ = @(t,x,u)system(x,u,zeros(Opts.nw,1));
    
    options=optimoptions('fmincon','MaxFunEvals',maxFunEvals,'Algorithm',...
                         'sqp','Display','off');  
                     
    x = fmincon(@(xc)costfunctionFmincon(xc,nx,N,xf_,Qext,R), ...
                xc0,[],[],[],[],[x_lb;u_lb],[x_ub;u_ub], ...
                @(xc)constraintfunctionFmincon(xc,nx,nu,N,h,x0,system_), ...
                options); 

     % extracting the optimal input trajectory
    u=x(nx*N+1:end);
    u=reshape(u,[nu,N]);
    
    % to obtain states from corner trajectory, we simulate the 
    % trajectory using the optimized center inputs with Runge-Kutta 45
    % integrator.
    x=zeros(nx,N+1);
    x(:,1) = x0;
    for k=1:N % integration stepwise since piece-wise constant control inputs
        [~,x_temp]=ode45(@(t,x)system(x,u(:,k),zeros(Opts.nw,1)),[0 h],x(:,k)');
        x(:,k+1) = x_temp(end,:)';
    end
