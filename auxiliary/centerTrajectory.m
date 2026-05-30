function [xCenter,uCenter] = centerTrajectory(sys,Opts)
% CENTERTRAJECTORY - Compute the reference trajectory
%
% Syntax:
%       [xCenter,uCenter] = CENTERTRAJECTORY(sys,Opts)
%
% Description:
%       This function computes the reference trajectory by solving an
%       optimal control problem, see Eq. 6 in the manual. If the user 
%       provided a reference trajectory then this trajectory is used.
%
% Input Arguments:
%
%       -sys:       function handle to the dynamics function of the
%                   benchmark
%       -Opts:      a structure containing the following settings
%
%           -.nx:       dimension of the state space
%                       (positive integer)
%           -.nu:       dimension of the control input space
%                       (positive integer)
%           -.nw:       dimension of the disturbance space
%                       (positive integer)
%           -.x0:       initial state
%                       (double array of dimension [nx,1])
%           -.xf:       goal state
%                       (double array of dimension [nx,1])
%           -.N:        number of center trajectory time-steps
%                       (positive integer)
%           -.dt:       size of a center trajectory time step
%                       (positive scalar)
%           -.Q:        state weighting matrix for the cost function
%                       (positive semi-definite matrix of dimension
%                       [nx,nx])
%           -.R:        input weighting matrix for the cost function
%                       (positive semi-definite matrix of dimension
%                       [nu,nu])
%           -.X:        set of admissible states
%                       (class: interval / polytope)
%           -.U:        set of admissible control inputs
%                       (class: interval / zonotope)
%           -.W:        set of disturbances
%                       (class: contSet)
%
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,N + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,N])
%
% See Also:
%       @optimalControl
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
% Authors:      Niklas Kochdumper, Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % check if center trajectory is provided by user
    if isfield(Opts,'refTraj') && isfield(Opts.refTraj,'u') && ...
       isfield(Opts.refTraj,'x')

        % check correctness of user input
        if size(Opts.refTraj.x,1) ~= Opts.nx
           error('Number of rows of Opts.refTraj.x has to be equal to the number of states!') 
        end
        if size(Opts.refTraj.x,2) ~= (Opts.N + 1)
           error('Number of columns of Opts.refTraj.x has to be equal to one plus the number of time steps!'); 
        end
        if size(Opts.refTraj.u,1) ~= Opts.nu
           error('Number of rows of Opts.refTraj.u has to be equal to the number of inputs!') 
        end
        if size(Opts.refTraj.u,2) ~= Opts.N
           error('Number of columns of Opts.refTraj.u has to be equal to the number of time steps'); 
        end
        
        xCenter = Opts.refTraj.x;
        uCenter = Opts.refTraj.u;

    else
        % generate center trajectory by solving an optimal control problem

        % ... define function handles for the cost function (see Eq. 6 in
        % the manual)
        stageCost = cell(Opts.N,1);
        stageCost(:) =  {@(x,u) Opts.dt*u'*Opts.R*u};
        terminalCost = @(x) (x-Opts.xf)'*Opts.Q*(x-Opts.xf);

        % optimal control
        % ... instantiate optimalControl object
        OCP = optimalControl(sys,stageCost,terminalCost,Opts.X,Opts.U,Opts.W,Opts.N,Opts.dt);
        % ... compute an initial guess by linearly interpolating between
        % the initial state x0 and the goal state xf
        y0 = OCP.initialGuessInterpolate(Opts.x0,Opts.xf);
        % solve optimal control problem
        sol = OCP.solveOCP(y0,Opts.x0);
        uCenter = sol.u;

        % simulate the center trajectory using the optimal solution
        xCenter = zeros(Opts.nx,Opts.N+1);
        xCenter(:,1) = Opts.x0;
        for kk=1:Opts.N
            [~,x_temp] = ode45(@(t,x)sys(x,uCenter(:,kk),zeros(Opts.nw,1)), ...
                [0 Opts.dt],xCenter(:,kk)');
            xCenter(:,kk+1) = x_temp(end,:)';
        end
    end
end