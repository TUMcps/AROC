function [param,res] = optimalControlReachsetMPC(x0,J,L,Opts)
% OPTIMALCONTROLREACHSETMPC - solve an optimal control problem
%
% Syntax:
%       [param,res] = OPTIMALCONTROLREACHSETMPC(x0,J,L,Opts)
%
% Description:
%       This function solves the optimal control problem as defined in 
%       Eq. (7) in [1] to determine a feasible reference trajectory.
%
% Input Arguments:
%
%       -x0:    initial state of the considered reference trajectory part
%               (dimension: [nx,1])
%       -L:     value of the objective function (=cost) of the previous
%               solution
%       -J:     summed distance of the points from the previous solution to
%               the terminal region 
%       -Opts:      a structure containing following options
%
%           -.funHandle:    function handle to the dynamic function
%           -.nx:           number of system states
%           -.nu:           number of system inputs
%           -.uMax:         upper bound for the input constraints
%           -.uMin:         lower bound for the input constraints
%           -.xf:           goal state
%           -.N:            number of time steps for the prediction 
%                           horizon. Prediction horizon: Opts.N * Opts.dT
%                           [{10} / positive integer]
%           -.dT:           time step. Prediction horizon: Opts.N * Opts.dT 
%           -.Q:            state weighting matrix for the cost function
%                           (center trajectory)
%           -.R:            input weighting matrix for the cost function
%                           (center trajectory)
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope)
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%           -.maxIter:      maximum number of iterations for the optimal
%                           control problem [{10} / positive integer]
%
% Output Arguments:
%
%       -param: a structure containing following options
%
%           -.xc:       reference trajectory (dimension: [nx,N+1])   
%           -.uc:       reference trajectory control inputs
%                       (dimension: [nu,N])
%           -.J:        summed distance of the reference trajectory points
%                       from the terminal region
%           -.L:        value of the objective function (=cost) of the 
%                       reference trajectory 	
%       -res:           flag that specifies if the computed reference
%                       trajectory is feasible (0 or 1)
%
% See Also:
%       optimalControl, reachsetMPC
%
% References:
%       * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
%              for Disturbed Nonlinear Systems
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

    % check if ACADO or fmincon is used for optimization
    if Opts.useAcado
       [u,x] = optimalControlAcado(Opts.funHandle,x0,J,Opts); 
    else
       [u,x] = optimalControlFmincon(Opts.funHandle,x0,J,Opts); 
    end
    
    % check if optimal control problem found a feasible solution
    [res,L,J] = checkConstraint(x,u,L,J,Opts);
    
    % store parameters
    param.xc = x;
    param.uc = u;
    param.L = L;
    param.J = J;
end



% Auxiliary Functions -----------------------------------------------------

function [u,x] = optimalControlAcado(sys,x0,J,Opts)

    % passing optimization variables to ACADO Toolbox, every scalar has to
    % be passed individually
    A_ = num2cell(Opts.termReg.A');
    b_ = num2cell(Opts.termReg.b);
    x0_ = num2cell(x0);
    x_f_ = num2cell(Opts.xf);
    Q_ = num2cell(Opts.Q');
    R_ = num2cell(Opts.R');
    u_max_ = num2cell(Opts.uMax_);
    u_min_ = num2cell(Opts.uMin_);
    
    alphaPoly = 1/(1+Opts.alpha);

    evalc(['out = acadoReachsetMPC_RUN(x0_{:},x_f_{:},Q_{:},R_{:},1,', ...
          'Opts.dT,u_max_{:},u_min_{:},A_{:},b_{:},alphaPoly,J,Opts.alpha);']);

    % saving optimal inputs for center trajectory which were computed in
    % optimal control problem using ACADO
    t_CONTROLS = out.CONTROLS(:,1)';
    uTemp = out.CONTROLS(:,2:end)';
    
    % restructure u to the correct format
    u = zeros(Opts.nu,Opts.N);
    index = 1;
    for i = 1:Opts.N
       u(:,i) = uTemp(index:index+Opts.nu-1,1:end-1);
       index = index + Opts.nu;
    end

    % to obtain states from center trajectory, we simulate the corner
    % trajectory using the optimized corner inputs with Runge-Kutta 45
    % integrator. This is done because the state determined by ACADO can
    % by inaccurate sometimes. 
    delta_t = t_CONTROLS(1,2)-t_CONTROLS(1,1);
    x = zeros(Opts.nx,size(u,2)+1);
    x(:,1) = x0;

    % integration stepwise since piece-wise constant control inputs
    funHan = @(x,u)sys(x,u,zeros(Opts.nw,1));
    
    for k = 1:size(u,2)
        [~,x_temp]=ode45(@(t,x)funHan(x,u(1:Opts.nu,k)),[0 delta_t],x(:,k)');
        x(:,k+1)=x_temp(end,:)';
    end    
end

function [u,x] = optimalControlFmincon(sys,x0,J,Opts)
    
    % enlarged input cost matrix for easier computation
    R = Opts.dT * kron(eye(Opts.N),Opts.R);

    % initialization of initial solution for optimal control problem by
    % integrating the states for very small control inputs
    u0 = diag(center(Opts.U))*ones(Opts.nu,Opts.N);
    xc0 = zeros(Opts.nx,Opts.N);
    xc0 = [x0,xc0];
    for k = 1:Opts.N
        [~,x_temp] = ode45(@(t,x)sys(x,u0(:,k),zeros(Opts.nw,1)),[0 Opts.dT],xc0(:,k)');
        xc0(:,k+1) = x_temp(end,:)';
    end
    
    % combine state and input trajectory as initial solution for optimal
    % control problem
    xc0 = reshape(xc0(:,2:end),[Opts.nx*Opts.N,1]);
    u0 = reshape(u0,[Opts.nu*Opts.N,1]);
    xc0 = [xc0;u0];

    % lower and upper bounds for states and inputs
    u_lb = repmat(Opts.uMin_,[Opts.N,1]);
    u_ub = repmat(Opts.uMax_,[Opts.N,1]);
    x_lb = -inf*ones(Opts.nx*Opts.N,1);    % no state constraints
    x_ub = inf*ones(Opts.nx*Opts.N,1);

    % solve optimal control problem using a multiple shooting algorithm and
    % fmincon for optimization
    system_ = @(x,u) sys(x,u,zeros(Opts.nw,1));
    alphaPoly = 1/(1+Opts.alpha);
    
    options=optimoptions('fmincon','MaxIterations',Opts.maxIter,'Algorithm',...
                         'sqp','Display','off');  
                     
    x = fmincon(@(xc)costFunFminconReachsetMPC(xc,Opts.nx,Opts.N,Opts.xf,Opts.Q,R), ...
                xc0,[],[],[],[],[x_lb;u_lb],[x_ub;u_ub], ...
                @(xc)conFunFminconReachsetMPC(xc,Opts.nx,Opts.nu,Opts.N,...
                     Opts.dT,x0,system_,Opts.termReg.A,Opts.termReg.b,Opts.alpha,J,alphaPoly),options); 

     % extracting the optimal input trajectory
    u = x(Opts.nx*Opts.N+1:end);
    u = reshape(u,[Opts.nu,Opts.N]);
    
    % to obtain states from corner trajectory, we simulate the 
    % trajectory using the optimized center inputs with Runge-Kutta 45
    % integrator.
    x = zeros(Opts.nx,Opts.N+1);
    x(:,1) = x0;
    for k = 1:Opts.N 
        [~,x_temp] = ode45(@(t,x)sys(x,u(:,k),zeros(Opts.nw,1)),[0 Opts.dT],x(:,k)');
        x(:,k+1) = x_temp(end,:)';
    end
end

function [c,ceq] = conFunFminconReachsetMPC(y,nx,nu,N,dt,x0,dynamics,A,b,alpha,J,alphaPoly)
% constraint function for the FMINCON optimization

    % extract states and inputs
    x = y(1:nx*N);
    u = y(nx*N+1:end);
    x = reshape(x,[nx,N]);
    u = reshape(u,[nu,N]);
    x = [x0,x];
    xconstr = zeros(nx,N);

    % constraint due to system dynamics
    for k=1:N
        [~,x_temp]=ode45(@(t,x)dynamics(x,u(:,k)),[0 dt],x(:,k)');
        xconstr(:,k) = x_temp(end,:)';
    end
    
    ceq = y(1:nx*N)-reshape(xconstr,[N*nx,1]); 

    % contraction constraint
    b = alphaPoly * b;
    dist = zeros(N,1);
    
    for i = 1:N
        temp = (A*xconstr(:,i) - b)./b;
        dist(i) = max(0,min(temp > 0));
    end

    c = sum(dist) - J + alpha;
end

function cost = costFunFminconReachsetMPC(y,nx,N,xf,Q,R)
% cost function for the FMINCON optimization

    % obtaining states and inputs
    x=y(nx*(N-1)+1:nx*N);
    u=y(nx*N+1:end);

    % cost function based on sum of used inputs and difference of final state
    % to desired final state
    cost=u'*R*u + (x-xf)'*Q*(x-xf);
    
end
