classdef optimalControl
    %OPTIMALCONTROL - Class prodiving the functionality for solving 
    %       nonlinear optimal control problems.
    %
    % Syntax:
    %       obj = optimalControl(system,stageCost,terminalCost,X,U,Q,N,dt)
    %
    % Description:
    %       This class provides the functionality required for solving
    %       nonlinear optimal control problems of the form:
    %           min     sum_k=0^{N-1} l_k(x_k,u_k) + V(x_N),
    %           s.t.    x0 = x_0,
    %                   \forall k \in {0,...,N-1}: x_{k+1} = f(x_k,u_k,0),
    %                   \forall k \in {0,...,N}: x_k \in X,
    %                   \forall k \in {0,...,N}: u_k \in U,
    %       where the stage cost functions l_k(x_k,u_k) might be
    %       time-varying, x0 denotes a given initial state, and the sets of
    %       admissible states and control inputs X and U, respectively, are
    %       assumed to be representable as polytopes.
    %       We use IPOPT [1] interfaced via CasADi [2] for solving the 
    %       optimal control problem.
    %
    % Input Arguments:
    %       -system:        function handle for the continuous-time
    %                       dynamics of the controlled system
    %       -stageCost:     cell array storing function handles for the 
    %                       time-varying stage cost functions 
    %                       (cell array of dimension [N,1])
    %       -terminalCost:  function handle for the terminal cost function
    %       -X:             set of admissible states
    %                       (class: interval / polytope)
    %       -U:             set of admissible control inputs
    %                       (class: interval / zonotope)
    %       -W:             set of disturbances
    %                       (class: contSet)
    %       -N:             number of time steps
    %                       (integer)
    %       -dt:            sampling time 
    %                       (scalar double)
    %       -opts:          structure with the following fields
    %           -.nw:           dimension of the disturbance space
    %                           (integer)
    %
    % Output Arguments:
    %       -obj:   generated instance of the class optimalControl
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] A. Wächter and L. T. Biegler, 'On the Implementation of a 
    %           Primal-Dual Interior Point Filter Line Search Algorithm for 
    %           Large-Scale Nonlinear Programming', 
    %           Mathematical Programming 106(1), pp. 25-57, 2006 
    %       [2] J. A. E. Andersson et al., 'CasADi – A software framework 
    %           for nonlinear optimization and optimal control', 
    %           Mathematical Programming Computation, vol. 11, no. 1, 
    %           pp. 1–36, 2019.
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
    
    properties
        constraintTolerance (1,1) double {mustBePositive} = 1e-6 % tolerance for constraint satisfaction
        softConPenaltyWeight (1,1) double {mustBePositive} = 1e3 % weight for exact penalty terms

        N (1,1) {mustBeInteger, mustBePositive} = 10 % number of time steps
        dt (1,1) double {mustBeReal, mustBePositive} = 0.1 % sampling time

        idxVars (1,1) struct = struct() % structure storing the indices of the optimization variables
        idxCon (1,1) struct = struct() % structure storing the indices of the constraint functions
        
        % cost function
        stageCost cell {mustBeUnderlyingType(stageCost,'cell')} = {} % cell array storing function handles for the (time-varying) stage cost functions
        terminalCost function_handle % function handle for the terminal cost function

        % dynamical system
        dynamics_dt function_handle % function handle for the time-discretized dynamics of the controlled system
        nx (1,1) {mustBeInteger, mustBePositive} = 1 % dimension of the state space
        nu (1,1) {mustBeInteger, mustBePositive} = 1 % dimension of the control input space

        % optimization
        solver % CasADi NLP solver object
    end
    
    methods
        function obj = optimalControl(system,stageCost,terminalCost,X,U,W,N,dt)
            %OPTIMALCONTROL Construct an instance of this class
            
            % set properties
            obj.nx = X.dim;
            obj.nu = U.dim;
            obj.N = N;
            obj.dt = dt;
            assert(length(stageCost) == obj.N, ['Length of cell array ' ...
                'storing the (time-varying) stage cost functions must be ' ...
                'equal the length of the time horizon!'])
            obj.stageCost = stageCost;
            obj.terminalCost = terminalCost;
            
            % time-discretize nominal dynamical system
            integrator = discretizeDynamics.RK4.getFunHandle();
            w_ref = zeros(W.dim,1);
            assert(contains(W,w_ref), ...
                'Disturbance set is assumed to contain the origin!')
            obj.dynamics_dt = @(x,u) integrator(x,u,w_ref,system,obj.dt);

            % check constraint sets
            % ... set of admissible states X
            if representsa(X,'interval') || representsa(X,'fullSpace')
                X = polytope(X);
            elseif ~representsa(X,'polytope')
                error(['Invalid representation of the set of ' ...
                    'admissible states - admissible set representations:' ...
                    'interval or polytope.'])
            end
            % ... set of admissible control inputs U
            if representsa(U,'emptySet')
                error('Set of admissible states must not be empty!')
            elseif representsa(U,'interval') || representsa(U,'zonotope')
                U = polytope(U);
            else
                error(['Invalid representation of the set of ' ...
                    'admissible control inputs - admissible set representations:' ...
                    'interval or zonotope.'])
            end

            % compute the indices of the OCP variables in the vector of 
            % optimization variables
            obj = computeVariableIndices(obj,U);

            % compute the indices of the OCP constraints in the
            % vector-valued constraint function
            obj = computeConstraintIndices(obj,X,U);

            % % setup CasADi NLP solver
            obj = setupSolver(obj,X,U);

        end
        
    end

    methods (Access=protected)
        % computation of variable indices
        obj = computeVariableIndices(obj,U)

        % computation of constraint indices
        obj = computeConstraintIndices(obj,X,U)
    end
end
