function [R,H] = linearBackwardReach(goalSet,xCenter,uCenter,U,Opts)
% LINEARBACKWARDREACH - backward reachable set for linearized system
%
% Syntax:
%       [R,H] = LINEARBACKWARDREACH(goalSet,xCenter,uCenter,U,Opts)
%
% Description:
%       This function computes the backward reachable set starting from the 
%       goal set  for the inearized system.
%
% Input Arguments:
%
%       -goalSet:       goal set that should be reached (class: zonotope)
%       -xCenter:       cell-array containing the reference trajectory
%                       states for all intermediate time steps
%       -uCenter:       cell-array containing the reference trajectory
%                       inputs for all intermediate time steps
%       -U:             cell-array containing the sets of admissble control
%                       inputs for all intermediate time steps
%       -Opts:          structure containing the following options
%
%           -.Ninter:       number of intermediate time steps
%           -.nu:           number of inputs
%           -.nw:           number of disturbances
%           -.dt:           time step size for one intermediate time step
%           -.funHandle:    function handle for the dynamic function
%           -.linDyn.A:     function handle to the system matrix of the 
%                           linearize dynamics
%           -.linDyn.B:     function handle to the input matrix of the 
%                           linearize dynamics
%
% Output Arguments:
%
%       -R:     backward reachable set (class: zonotope)
%       -H:     matrix storing the generator-to-input assignment
%
% See Also:
%       safetyNetControl
%
% References:
%       * *[1] Schuermann et al. (2019)*, Formal Safety Net Control Using
%              Backward Reachability Analysis
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % loop backwards over all intermediate time steps
    R = goalSet;

    for i = Opts.Ninter:-1:1     

        % compute linearized system dynamics
        p = 0.5*(xCenter(:,i)+xCenter(:,i+1));
        uRef = uCenter(:,i);

        cc = Opts.funHandle(p,uRef,zeros(Opts.nw,1));
        Ac = Opts.linDyn.A(p,uRef);
        Bc = Opts.linDyn.B(p,uRef);

        % compute system matrices for the discrete time system
        Ainv = expm(-Ac*Opts.dt);

        temp = integral(@(t)(expm(Ac*(Opts.dt-t))),0,Opts.dt, ...
                        'ArrayValued',true);   
        B = temp*Bc;
        c = temp*(cc - Ac*p - Bc*uRef);

        % compute backward reachable set
        R = Ainv*(R + (-1)*B*U{i} - c);
    end

    % generate matrix H that stores the generator-to-input assignment
    dim = Opts.Ninter*Opts.nu;
    gen = size(goalSet.Z,2)-1;

    H = [zeros(dim,gen),eye(dim)];
end