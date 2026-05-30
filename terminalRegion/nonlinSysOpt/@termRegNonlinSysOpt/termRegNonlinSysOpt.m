classdef termRegNonlinSysOpt < terminalRegion
    %TERMREGNONLINSYSOPT - abstract superclass for terminal region objects
    %       computed with the nonlinSysApproach algorithms.
    %
    % Syntax:
    %       obj = termRegNonlinSysOpt(dynamics,sysType,set,dt,U,W)
    %
    % Description:
    %       Abstract superclass for terminal regions computed with the 
    %       linearization- [1] or polynomialzation-based [2] approach for
    %       computing terminal region of nonlinear sytems. Its
    %       childclasses provide methods for evaluating the respective
    %       invariance-preserving control law.
    %
    % Input Arguments:
    %       -dynamics:  handle to the dynamics function of the nonlinear
    %                   open-loop system
    %       -sysType:   type of dynamical system, e.g., sampled-data system
    %                   or discrete-time system 
    %                   (class: termRegSysType)
    %       -set:       set representing the terminal region 
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
    %               termRegNonlinSysOpt
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
    %           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
    %           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
    %       [2] L. Schäfer and M. Althoff, "Computing Robust Control 
    %           Invariant Sets of Nonlinear Systems Using Polynomial 
    %           Controller Synthesis," American Control Conference, 2024, 
    %           pp. 4162-4169. 
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
        dt (1,1) double {mustBePositive} = 0.1 % sampling time
        Ninter (1,1) {mustBeInteger} = 1 % number of intermediate control steps
        sysType termRegSysType = termRegSysType.SD % system type
    end
    
    methods
        function obj = termRegNonlinSysOpt(dynamics,sysType,set,dt,U,W)
            %TERMREGNONLINSYSOPT Construct an instance of this class

            % verify that set is representable as a zonotope
            assert(representsa(set,'zonotope'),['Terminal region must be ' ...
                'representable as a zonotope!'])
    
            % call superclass constructor
            Params.U = U;
            Params.W = W;
            obj = obj@terminalRegion(dynamics,set,Params);

            % set parameters
            obj.dt = dt;
            obj.sysType = sysType;
        end
    end
end
