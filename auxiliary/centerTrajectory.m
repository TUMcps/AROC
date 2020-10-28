function [xCenter,uCenter] = centerTrajectory(sys,Opts)
% CENTERTRAJECTORY - Compute the reference trajectory
%
% Syntax:
%       [xCenter,uCenter] = CENTERTRAJECTORY(sys,Opts)
%
% Description:
%       This function computes the reference trajectory by solving an
%       optimal control problem. If the user provided a reference
%       trajectory then this trajectory is used.
%
% Input Arguments:
%
%       -sys:       function handle to the dynamic function of the
%                   benchmark
%       -Opts:      a structure containing following options
%
%           -.nx:       number of system states
%           -.nu:       number of system inputs
%           -.x0:       initial state
%           -.xf:       goal state
%           -.Nc:       number of center trajectory time-steps
%           -.hc:       time step size for one center trajectory time step
%           -.Q:        state weighting matrix for the cost function
%           -.R:        input weighting matrix for the cost function
%
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,Nc + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Nc])
%
% See Also:
%       optimalControl, optimalControlFmincon
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

    % check if center trajectory is provided by user
    if isfield(Opts,'refTraj') && isfield(Opts.refTraj,'u') && ...
       isfield(Opts.refTraj,'x')

        % check correctness of user input
        if size(Opts.refTraj.x,1) ~= Opts.nx
           error('Number of rows of Opts.refTraj.x has to be equal to the number of states!') 
        end
        if size(Opts.refTraj.x,2) ~= (Opts.Nc + 1)
           error('Number of columns of Opts.refTraj.x has to be equal to one plus the number of time steps!'); 
        end
        if size(Opts.refTraj.u,1) ~= Opts.nu
           error('Number of rows of Opts.refTraj.u has to be equal to the number of inputs!') 
        end
        if size(Opts.refTraj.u,2) ~= Opts.Nc
           error('Number of columns of Opts.refTraj.u has to be equal to the number of time steps'); 
        end
        
        xCenter = Opts.refTraj.x;
        uCenter = Opts.refTraj.u;

    else

        % generate center trajectory by solving an optimal control problem
        if Opts.useAcado
            [uCenter,xCenter] = optimalControl(sys,Opts.xf,Opts.x0,Opts.hc, ...
                Opts.Q,Opts.R,Opts.Nc,1,Opts);
        else
            [uCenter,xCenter] = optimalControlFmincon(sys,Opts.xf,Opts.x0,Opts.hc, ...
                Opts.Q,Opts.R,Opts.Nc, ...
                0.01,50000,1,Opts);
        end
    end
end