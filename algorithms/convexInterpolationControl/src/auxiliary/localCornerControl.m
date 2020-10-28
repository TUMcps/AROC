function [uTotal, xt] = localCornerControl(system,xf,vert,h,Q,R,steps,lenHorizon,Opts)
% LOCALCORNERCONTROL - Solve optimal control problem for the corner states 
%                      of the parallelotope
%
% Syntax:
%       [uTotal, xt]= LOCALCORNERCONTROL(system,xf,vert,h,Q,R,steps,lenHorizon,Opts)
%
% Description:
%       This function solves an optimal control problem for all vertices
%       of the initial parallelotope (see Lines 6-8 of Alg. 1 in [1]). 
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
%       -h:             length of one timestep of the multiple shooting 
%                       algorithm
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
%       -Opts:          a structure containing following options
%           -.parallel:         boolean value that determines if parallel
%                               computing should be used or not (0 or 1)
%           -.useAcado:         use ACADO toolbox for solving the optimal 
%                               control problem. Fmincon is used otherwise 
%                               (0 or 1)
%           -.extHorizon.decay: decay function for the objective
%                               function of the optimization problem
%                               with extended optimization horizon
%                               ['uniform' / 'fall' / 'fall+end' / 
%                                'fallLinear' / 'fallLinear+End' / 
%                                'fallEqDiff' / 'FallEqDiff+End' / 
%                                'rise' / 'quad' /  'riseLinear' /
%                                'riseEqDiff' / 'end']
%
% Output Arguments:
%
%       -uTotal:        optimal control inputs for the corner trajectories
%                       (dimension: [nu,2^nx,steps*lenHorizon])
%       -xt:            resulting corner trajectories
%                       (dimension: [nx,2^nx,steps*lenHorizon])
%
% See Also:
%       convexInterpolationControl, optimalControl, optimalControlFmincon
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


    % Initialize variables

    nx = Opts.nx;       % number of state dimensions
    nu = Opts.nu;       % number of input dimensions
    hc = h;             % time step size
    n_v = size(vert,2); % number of extreme points

    uTotal=zeros(nu,n_v,lenHorizon*steps);    % vector to save the optimized inputs
    xt=zeros(nx,n_v,lenHorizon*steps+1);       % vector to save the predicted states

    % Solve optimal control problem for each extreme point

    if Opts.useAcado
        if Opts.parallel
            parfor i=1:1:n_v 
                [uTotal(:,i,:),xt(:,i,:)] = optimalControl(system,xf, ...
                                   vert(:,i),hc,Q,R,steps,lenHorizon,Opts);
            end
        else
            for i=1:1:n_v 
                [uTotal(:,i,:),xt(:,i,:)] = optimalControl(system,xf, ...
                                   vert(:,i),hc,Q,R,steps,lenHorizon,Opts);
            end
        end
    else
        if Opts.parallel
            parfor i=1:1:n_v              
                [uTotal(:,i,:),xt(:,i,:)] = optimalControlFmincon(system, ...
                       xf,vert(:,i),hc,Q,R,steps,0,70000,lenHorizon,Opts);                
            end
        else
            for i=1:1:n_v                  
                [uTotal(:,i,:),xt(:,i,:)] = optimalControlFmincon(system, ...
                       xf,vert(:,i),hc,Q,R,steps,0,70000,lenHorizon,Opts);
            end
        end
    end
    
