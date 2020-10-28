function [A,B,c,xf] = linearizeSystem(xCenter, uCenter, Opts)
% LINEARIZESYSTEM - compute linearized discrete time systems
%
% Syntax:
%       [A,B,c,xf] = LINEARIZESYSTEM(xCenter,uCenter,Opts)
%
% Description:
%       This function computes the linearized discrete time system for all
%       time steps along the reference trajectory
%       
%
% Input Arguments:
%
%       - xCenter:  reference trajectory states
%       - uCenter:  reference trajectory inputs
%       - Opts:     a struct containing the following fields
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
%       - A:    cell-array storing the system matrices of the linearized
%               time discrete system x(k+1) = A x(k) + B u(k) + c
%       - B:    cell-array storing the input matrices of the linearized
%               time discrete system x(k+1) = A x(k) + B u(k) + c
%       - c:    cell-array storing the constant offset of the linearized
%               time discrete system x(k+1) = A x(k) + B u(k) + c
%       - xf:   cell-array storing the goal states for each time step
%                   
% See Also:
%       generatorSpaceControl
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
% Authors:      Jan Wagener, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization 
    A = cell(Opts.N,1); B = cell(Opts.N,1); c = cell(Opts.N,1);
    xf = cell(Opts.N,1);
    
    counter = 1;
    
    % loop over all time steps
    for i = 1:Opts.N
       
        A{i} = cell(Opts.Ninter,1);
        B{i} = cell(Opts.Ninter,1);
        c{i} = cell(Opts.Ninter,1);
        
        % loop over all intermediate time steps
        for j = 1:Opts.Ninter
            
            % compute linearization point
            x_ = 0.5*(xCenter(:,counter)+xCenter(:,counter+1));
            u_ = uCenter(:,counter);
            counter = counter + 1;
            
            % compute linearized system dynamics
            cc = Opts.funHandle(x_,u_,zeros(Opts.nw,1));
            Ac = Opts.linDyn.A(x_,u_);
            Bc = Opts.linDyn.B(x_,u_);

            % compute system matrices for the discrete time system
            A{i}{j} = expm(Ac*Opts.dt);

            temp = integral(@(t)expm(Ac*t),0,Opts.dt, ...
                            'ArrayValued',true);   
            B{i}{j} = temp*Bc;
            c{i}{j} = temp*(cc - Ac*x_ - Bc*u_);
        end
        
        % store the goal state
        xf{i} = xCenter(:,counter);
    end