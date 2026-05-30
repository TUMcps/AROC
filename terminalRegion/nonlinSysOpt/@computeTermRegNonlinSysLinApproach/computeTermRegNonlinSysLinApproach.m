classdef computeTermRegNonlinSysLinApproach < computeTermRegNonlinSysOpt
    %COMPUTETERMREGNONLINSYSLINAPPROACH - solver class for the
    %       linearization-based approach for computing terminal regions of
    %       nonlinear systems.
    %
    % Syntax:
    %       obj = computeTermRegNonlinSysOpt(Param,Opts)
    %
    % Description:
    %       This class and its childclasses provide all methods required
    %       for formulating and solving the convex approximations of the 
    %       terminal region optimization problem in [1, Sec. III] as well 
    %       as the methods for verifying correctness of its solution. 
    %
    % Input Arguments:
    %       -Param:         a struct containing the benchmark parameters
    %           .X:         set of admissible states
    %                       (class: interval/zonotope/polytope)
    %           .U:         set of admissible control inputs
    %                       (class: interval/zonotope/polytope)
    %           .W:         set of disturbances
    %                       (class: zonotope)
    %
    %       -Opts:          a struct containing the algorithm settings
    %           .xEq:                   equlibrium state to be stabilized
    %                                   (double vector)
    %           .uEq:                   equilibrium control input
    %                                   (double vector)
    %           .termRegOrder:          (optional) order of the zonotope 
    %                                   representing the terminal region
    %                                   (positive scalar, >= 1)
    %           .verbose:               if set to true, the algorithm prints 
    %                                   information about each iteration to 
    %                                   the command window. If set to 
    %                                   false, no iterative output is shown.
    %           -solverSettings:    (optional) struct storing the solver
    %                               settings, see superclass constructor 
    %                               for shared fields
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               computeTermRegNonlinSysLinApproach
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
    %           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
    %           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024.
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
    % Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
    %               Embedded Systems, TU Muenchen
    %------------------------------------------------------------------
        
    methods
        function obj = computeTermRegNonlinSysLinApproach(Param,Opts)

            % check optional input arguments
            if isfield(Opts,'solverSettings')
                solverSettings = Opts.solverSettings;
            else
                solverSettings = struct();
            end
            % call superclass constructor
            obj = obj@computeTermRegNonlinSysOpt( ...
                Param.X,Param.U,Param.W, ...
                Opts.xEq,Opts.uEq,Opts.termRegOrder,Opts.verbose,solverSettings);
        end

        function obj = updateCostSet(obj,s)
            % Given the optimized set of scaling factors of the set whose
            % volume is optimized, these sets generators are scaled.
            %
            % Input Argument:
            %       s:  (optimal) generator scaling factors
            %           (double vector)
            %
            % Output Argument:
            %       -obj:   instance of this class with scaled cost set
            %
            % ------------------------------------------------------------

            obj.costSet = zonotope(obj.costSet.c, obj.costSet.G*diag(s));

        end
    end

    methods(Static)

        % constraint function: evaluation of the parameterized
        % approximation of the abstraction error
        approxAbsErr = approxAbsErrQuad_eval(var_dz,var_exploitTR,quadCoeff,bilinCoeff,trCoeff)

        % soft formulation of the zonotope-in-zonotope containment problem
        [cEq,cIneq] = cvx_zonoContSoftConstraint(cIn,GIn,cOut,GOut,Gamma,gamma,Gamma_abs,gamma_abs,ub,slack)

        % returns the semi-edge lengths of the interval enclosure of the
        % parameterized terminal region
        IH_semiEdgeLength = cvx_intervalTermReg(var_s,termReg)
        
        % constraint function implementing the necessary and sufficient
        % conditions for zonotope-in-polytope containment
        cIneq = cvx_zonoInPolyConstraint(cIn,GIn,s,polyOut)
    end
end
