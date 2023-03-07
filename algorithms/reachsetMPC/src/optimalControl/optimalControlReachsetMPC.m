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
% solver optimal control problem using ACADO

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

    if ~isempty(Opts.X)
        C = num2cell(Opts.X_.P.A);
        d = num2cell(Opts.X_.P.b);
    else
        C = num2cell([]); d = num2cell([]);
    end

    evalc(['out = acadoReachsetMPC_RUN(x0_{:},x_f_{:},Q_{:},R_{:},Opts.Ninter,', ...
          'Opts.dT,u_max_{:},u_min_{:},A_{:},b_{:},alphaPoly,J,Opts.alpha,C{:},d{:});']);

    % saving optimal inputs for center trajectory which were computed in
    % optimal control problem using ACADO
    t_CONTROLS = out.CONTROLS(:,1)';
    uTemp = out.CONTROLS(:,2:end)';
    
    % restructure u to the correct format
    u = cell(Opts.N,1);
    index = 1;
    
    for i = 1:Opts.N
        u{i} = uTemp(index:index+Opts.nu-1,1:end-1);
        index = index + Opts.nu;
    end

    % to obtain states from center trajectory, we simulate the corner
    % trajectory using the optimized corner inputs with Runge-Kutta 45
    % integrator. This is done because the state determined by ACADO can
    % by inaccurate sometimes. 
    dt = t_CONTROLS(1,2)-t_CONTROLS(1,1);
    x = cell(Opts.N,1);

    % integration stepwise since piece-wise constant control inputs
    funHan = @(x,u)sys(x,u,zeros(Opts.nw,1));
    
    for i = 1:Opts.N
        
        x{i} = zeros(Opts.nx,Opts.Ninter+1);
        x{i}(:,1) = x0;
        
        for k = 1:Opts.Ninter
            [~,x_temp] = ode45(@(t,x)funHan(x,u{i}(1:Opts.nu,k)),...
                               [0 dt],x{i}(:,k)');
            x{i}(:,k+1) = x_temp(end,:)';
        end    
        
        x0 = x_temp(end,:)';
    end
end

function [u,x] = optimalControlFmincon(sys,x0,J,Opts)
% Solve optimal control problem using fmincon

    % solve optimal control problem
    inter = 4;
    N = Opts.N * Opts.Ninter;
    alphaPoly = 1/(1+Opts.alpha);
    
    if ~isempty(Opts.X)
        text = sprintf(['cost = @(x) fminconReachsetMPC_cost(x,x0,', ...
                        'Opts.xf,Opts.Q,Opts.R,Opts.dT/Opts.Ninter,', ...
                        'Opts.termReg.A,Opts.termReg.b,Opts.alpha,', ...
                        'alphaPoly,J,Opts.uMin_,Opts.uMax_,', ...
                        'Opts.X_.P.A,Opts.X_.P.b);']);
        eval(text);
    
        text = sprintf(['con = @(x) fminconReachsetMPC_con(x,x0,', ...
                        'Opts.xf,Opts.Q,Opts.R,Opts.dT/Opts.Ninter,', ...
                        'Opts.termReg.A,Opts.termReg.b,Opts.alpha,', ...
                        'alphaPoly,J,Opts.uMin_,Opts.uMax_,', ...
                        'Opts.X_.P.A,Opts.X_.P.b);']);
        eval(text);
    else
        text = sprintf(['cost = @(x) fminconReachsetMPC_cost(x,x0,', ...
                        'Opts.xf,Opts.Q,Opts.R,Opts.dT/Opts.Ninter,', ...
                        'Opts.termReg.A,Opts.termReg.b,Opts.alpha,', ...
                        'alphaPoly,J,Opts.uMin_,Opts.uMax_);']);
        eval(text);
    
        text = sprintf(['con = @(x) fminconReachsetMPC_con(x,x0,', ...
                        'Opts.xf,Opts.Q,Opts.R,Opts.dT/Opts.Ninter,', ...
                        'Opts.termReg.A,Opts.termReg.b,Opts.alpha,', ...
                        'alphaPoly,J,Opts.uMin_,Opts.uMax_);']);
        eval(text);
    end
    
    options = optimoptions('fmincon','Algorithm', ...
                            'interior-point','Display','off', ...
                            'MaxFunctionEvaluations',1e4); 
                        
    nOpt = Opts.nx*(N*inter+1) + Opts.nu*N + Opts.N;
    xInit = zeros(nOpt,1);
    
    lb = -inf*ones(nOpt,1);
    ub = inf*ones(nOpt,1);
    
    cnt = Opts.nx*(N*inter+1);
    lb(cnt+1:cnt+Opts.nu*N) = repmat(Opts.uMin,[N,1]);
    ub(cnt+1:cnt+Opts.nu*N) = repmat(Opts.uMax,[N,1]);
    
    lb(cnt+Opts.nu*N+1:end) = zeros(nOpt - (cnt + Opts.nu*N), 1);
    
    sol = fmincon(cost,xInit,[],[],[],[],lb,ub,con,options);

    x_ = sol(1:Opts.nx*(N*inter+1));
    x_ = reshape(x_,[Opts.nx,N*inter+1]);
    u = sol(numel(x_)+1:numel(x_)+Opts.nu*N);
    u = reshape(u,[Opts.nu,N]);
    
    % simulate trajectory to obtain more accurate result
    x = zeros(Opts.nx,Opts.N+1);
    x(:,1) = x0;
    for k = 1:N 
        [~,x_temp] = ode45(@(t,x)sys(x,u(:,k),zeros(Opts.nw,1)), ...
                                        [0 Opts.dT/Opts.Ninter],x(:,k)');
        x(:,k+1) = x_temp(end,:)';
    end
    
    % bring to correct format
    cnt = 1; u_ = cell(Opts.N,1); x_ = cell(Opts.N,1);
    for i = 1:Opts.N
       u_{i} = u(:,cnt:cnt + Opts.Ninter-1);
       x_{i} = x(:,cnt:cnt + Opts.Ninter);
       cnt = cnt + Opts.Ninter;
    end
    
    u = u_; x = x_;
end