function [u,x] = optimalControl(system,xf,x0,h,Q,R,steps,lenHorizon,Opts)
% OPTIMALCONTROL - Solve optimal control problem 
%
% Syntax:
%       [u,x] = OPTIMALCONTROL(system,xf,vert,h,Q,R,steps,lenHorizon)
%
% Description:
%       This function solves an optimal control problem 
%
% Input Arguments:
%
%       -system:        object containing the system dynamics (class:
%                       nonlinearSys) 
%       -xf:            desired final state at the end of the control
%                       process
%                       (dimension: [nx, 1])
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
%       -lenHorizon:    length of the optimization horizon in center
%                       trajectory time steps
%
% Output Arguments:
%
%       -u:             optimal control input
%                       (dimension: [nu,steps*lenHorizon])
%       -x:             resulting state trajectories
%                       (dimension: [nx,steps*lenHorizon])
%
% See Also:
%       convexInterpolationControl, localCornerControl
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
    nx = Opts.nx;
    nu = Opts.nu;

    % passing optimization variables to ACADO Toolbox, every scalar has to be
    % passed individually
    x0_ = num2cell(x0);
    x_f_ = num2cell(xf);
    Q_ = num2cell(Q');
    R_ = num2cell(R');
    u_max_ = num2cell(Opts.uMax);
    u_min_ = num2cell(Opts.uMin);

    evalc(sprintf('out = acado%i_RUN(x0_{:},x_f_{:},Q_{:},R_{:},steps,steps*h,u_max_{:},u_min_{:});',lenHorizon));
    

    % saving optimal inputs for corner trajectory which were computed in
    % optimal control problem using ACADO
    t_CONTROLS = out.CONTROLS(:,1)';
    uTemp = out.CONTROLS(:,2:end)';
    
    % restructure u to the correct format
    u = zeros(nu,steps*lenHorizon);
    index1 = 1;
    index2 = 1;
    for i = 1:lenHorizon
       u(:,index1:index1+steps-1) = uTemp(index2:index2+nu-1,1:end-1);
       index1 = index1 + steps;
       index2 = index2 + nu;
    end

    % to obtain states from corner trajectory, we simulate the corner
    % trajectory using the optimized corner inputs with Runge-Kutta 45
    % integrator. 
    delta_t = t_CONTROLS(1,2) - t_CONTROLS(1,1);
    x = zeros(nx,size(u,2)+1);
    x(:,1) = x0;

    % integration stepwise since piece-wise constant control inputs
    for k=1:size(u,2)
        [~,x_temp] = ode45(@(t,x)system(x,u(1:nu,k),zeros(Opts.nw,1)),[0 delta_t],x(:,k)');
        x(:,k+1) = x_temp(end,:)';
    end