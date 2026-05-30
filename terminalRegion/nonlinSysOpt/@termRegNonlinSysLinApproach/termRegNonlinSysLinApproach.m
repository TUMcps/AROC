classdef termRegNonlinSysLinApproach < termRegNonlinSysOpt
    %TERMREGNONLINSYSLINAPPROACH - terminal region object computed with the
    %       linearization-based approach for computing terminal regions of
    %       nonlinear sytems.
    %
    % Syntax:
    %       obj = termRegNonlinSysLinApproach(dynamics,sysType,set,ctrlSet,sysType,dt,U,W)
    % 
    % Description:
    %       This class represents a terminal region computed using the
    %       linearization-based [1] approach for computing terminal region 
    %       of nonlinear sytems.       
    %
    % Input Arguments:
    %       -dynamics:  handle to the dynamics function of the nonlinear
    %                   open-loop system
    %       -sysType:   type of dynamical system, e.g., sampled-data system
    %                   or discrete-time system 
    %                   (class: termRegSysType)
    %       -set:       set representing the terminal region 
    %                   (class: zonotope) 
    %       -ctrlSet:   set representing the invariance-preserving control
    %                   law
    %                   (class: zonotope)
    %       -dt:        sampling time
    %                   (positive scalar)
    %       -U:         set of admissible control inputs
    %                   (class: zonotope / polytope / interval)
    %       -W:         set of disturbances
    %                   (class: zonotope / interval))
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               termRegNonlinSysLinApproach
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
    %           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
    %           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
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

    
    properties (SetAccess = immutable, GetAccess = public)
        ctrlSet (1,1) contSet % zonotope representing the controller
    end
    
    methods
        function obj = termRegNonlinSysLinApproach(dynamics,sysType,set,ctrlSet,dt,U,W)
            %TERMREGNONLINSYSLINAPPROACH Construct an instance of this class
            
            % call superclass constructor 
            obj = obj@termRegNonlinSysOpt(dynamics,sysType,set,dt,U,W)

            % set parameters
            obj.ctrlSet = ctrlSet;
        end
    end

    methods(Access = public)
        function u = computeControlInput(obj,x)
            % Computes the invariance-preserving control input given a 
            % state x in the terminal region.
            %
            % Input Argument:
            %       -x:     current state of the dynamical system
            %               (double vector)
            %
            % Output Argument:
            %       -u:     invariance-preserving control input u(x) 
            %               (double vector)
            %
            % ------------------------------------------------------------
            
            % compute parameterization of x in the terminal region
            lambda = obj.computeStateParameterization(x);

            % compute control input
            u = obj.ctrlSet.c + obj.ctrlSet.G*lambda;
        end
    end
end
