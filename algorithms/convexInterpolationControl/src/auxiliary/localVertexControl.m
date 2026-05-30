function [uTotal, xt] = localVertexControl(sys,vertices,xref,M,Opts)
% LOCALVERTEXCONTROL - computes the input trajectories of the vertices of
%       the initial parallelotope.
%
% Syntax:
%       [uTotal, xt]= localVertexControl(system,xf,vert,h,Q,R,steps,lenHorizon,Opts)
%
% Description:
%       This function solves an optimal control problem for all vertices
%       of the initial parallelotope (see Lines 6-8 of Alg. 1 in [1]). 
%
% Input Arguments:
%
%       -sys:           function handle to the dynamics function of the
%                       benchmark
%       -xref:          reference trajectory 
%                       (double array of dimension [nx,M])
%       -vertices:      vertices of the initial parallelotope.
%                       (dimension: [x,2^nx])
%       -M:             number of reference trajectory time steps
%                       (positive integer)
%       -Opts:          a structure containing the following settings
%
%           -.nx:       dimension of the state space
%                       (positive integer)
%           -.nu:       dimension of the control input space
%                       (positive integer)
%           -.nw:       dimension of the disturbance space
%                       (positive integer)
%           -.dt:       size of a center trajectory time step
%                       (positive scalar)
%           -.Ninter:   number of intermediate time steps during one
%                       center trajectory time step
%                       (positive integer)
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
%           -.extHorizonWeightingFunction:  instance of enumeration class 
%                                           extendedHorizonWeights 
%                                           specifying the weighting 
%                                           function for the extended horizon 
%
% Output Arguments:
%
%       -uTotal:        optimal control inputs for the corner trajectories
%                       (double array of dimension [nu,2^nx,M*Ninter])
%       -xt:            resulting corner trajectories
%                       (double array of dimension [nx,2^nx,M*Ninter])
%
% See Also:
%       convexInterpolationControl, optimalControl
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% total number of time steps
N = M*Opts.Ninter;

% sampling time of the control inputs
dt_u = Opts.dt/Opts.Ninter; 

% function handles for the cost function (see Eq. 18 in the manual)
stageCost = cell(N,1);
% ... control input penalties
stageCost(:) =  {@(x,u) dt_u*u'*Opts.R*u};
% ... add reference tracking error
w = Opts.extHorizonWeightingFunction.computeWeights(M);
for kk = 1:M-1
    stageCost{kk*Opts.Ninter+1} = ...
        @(x,u) w(kk)* (x-xref(:,kk))'*Opts.Q*(x-xref(:,kk)) + dt_u*u'*Opts.R*u;
end  
% ... terminal cost
terminalCost = @(x) w(M)* (x-xref(:,M))'*Opts.Q*(x-xref(:,M));

% optimal control
% ... instantiate optimalControl object
OCP = optimalControl(sys,stageCost,terminalCost,Opts.X,Opts.U,Opts.W,N,dt_u);

% ... solve an optimal control problem for each vertex of the initial
% parallelotope
n_v = size(vertices,2);
uTotal=zeros(Opts.nu,n_v,N);    
xt=zeros(Opts.nx,n_v,N+1); 
xf = xref(:,M); % final state
for ii = 1:n_v
    % ... compute an initial guess by linearly interpolating between the
    % initial state x0 and the goal state xf
    x0 = vertices(:,ii);
    y0 = OCP.initialGuessInterpolate(x0,xf);
    % solve optimal control problem
    sol = OCP.solveOCP(y0,x0);
    uTotal(:,ii,:) = sol.u;

    % simulate the system using the optimized input trajectory
    x_sim = zeros(Opts.nx,N+1);
    x_sim(:,1) = x0;
    for kk=1:N
        [~,x_temp] = ode45(@(t,x)sys(x,uTotal(:,ii,kk),zeros(Opts.nw,1)), ...
            [0 dt_u],x_sim(:,kk)');
        x_sim(:,kk+1) = x_temp(end,:)';
    end
    xt(:,ii,:) = x_sim;
end

end
