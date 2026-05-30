classdef termRegNonlinSysPolyApproach < termRegNonlinSysOpt
    %TERMREGNONLINSYSPOLYAPPROACH - terminal region object computed with the
    %       polynomialization-based approach for computing terminal regions 
    %       of nonlinear sytems.
    %
    % Syntax:
    %       obj = termRegNonlinSysLinApproach(dynamics,set,ctrlSet,sysType,dt,U,W)
    % 
    % Description:
    %       This class represents a terminal region computed using the
    %       polynomialization-based [1] approach for computing terminal region 
    %       of nonlinear sytems.       
    %
    % Input Arguments:
    %       -dynamics:  handle to the dynamics function of the nonlinear
    %                   system
    %       -set:       set representing the terminal region 
    %                   (class: polyZonotope) 
    %       -ctrlSet:   set representing the invariance-preserving control
    %                   law
    %                   (class: polyZonotope)
    %       -sysType:   type of dynamical system must be a sampled-data 
    %                   system
    %                   (class: termRegSysType)
    %       -dt:        sampling time
    %                   (positive scalar)
    %       -U:         set of admissible control inputs
    %                   (class: zonotope / polytope / interval)
    %       -W:         set of disturbances
    %                   (class: zonotope / interval))
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               termRegNonlinSysPolyApproach
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer and M. Althoff, "Computing Robust Control 
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

    properties(SetAccess = immutable, GetAccess = public)
        ctrlSet (1,1) contSet % polyZonotope representing the controller
        ctrlParam double {mustBeVector} = 0 % vector storing the controller parameters
    end

    methods
        function obj = termRegNonlinSysPolyApproach(dynamics,sysType,set,ctrlSet,ctrlParam,dt,U,W)
            %TERMREGNONLINSYSPOLYAPPROACH Construct an instance of this class

            assert(isa(set,'polyZonotope') && isempty(set.GI) && ...
                size(set.E,1)==size(set.E,2) && isequal(set.E, eye(size(set.E))), ...
                ['Terminal region must be represented as a polynomialZonotope ' ...
                'with identity as exponential matrix.'])
            assert(sysType==termRegSysType.SD, ['Polynomialization-based ...' ...
                'algorithm for computing terminal regions is only implemented for sampled-data systems.'])
            % call superclass constructor 
            obj = obj@termRegNonlinSysOpt(dynamics,sysType,set,dt,U,W)

            % set parameters
            assert(representsa(ctrlSet,'polyZonotope'));
            obj.ctrlSet = ctrlSet;
            obj.ctrlParam = ctrlParam;

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
            u = obj.ctrlSet.resolve(lambda);
        end
    end
end