classdef termRegNonlinSysSolutionAuxData
    %TERMREGNONLINSYSSOLUTIONAUXDATA - dataclass for storing auxiliary data
    %       from the solution of the terminal region optimization problems.
    %
    % Syntax:  
    %       obj = termRegNonlinSysSolutionAuxData(flagIsFeasible,costSet,tr_exploit,estAbsErr,solver_time,opt_setupTime)
    %
    % Description:
    %       Dataclass for storing the auxiliary data from the solution of 
    %       the convex [1] or polynomial [2] approximations of the terminal 
    %       region optimization problem. Such data includes the estimated
    %       set of abstraction errors (required for updating the trust
    %       region) or the cost set (see also [1, Remark 2]).
    %
    % Input Arguments:
    %       -flagIsFeasible:    flag indicating feasibility of the solution
    %                           (logical)
    %       -costSet:           set, which is plugged into the cost
    %                           function (optimal solution)
    %                           (class: zonotope)
    %       -tr_exploit:        scalar indicating the exploitiation of the
    %                           trust region
    %       -estAbsErr:         estimate of the set of abstraction errors
    %                           (class: interval)
    %       -solver_time:       time required for solving the optimization 
    %                           problem
    %                           (nonnegative scalar)
    %       -opt_setupTime:     time required for setting up the 
    %                           optimization problem
    %                           (nonnegative scalar)
    %
    % Output Arguments:
    %       -obj:       instance of class termRegNonlinSysSolutionAuxData
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
    
    properties (GetAccess=public, SetAccess=immutable)
        flagIsFeasible (1,1) logical = false % flag indicating feasibility of the solution
        costSet zonotope = zonotope.empty(1) % set, which is plugged into the cost fuction, see [1, Remark 2]
        trustRegionExploitation (1,1) double {mustBeInRange(trustRegionExploitation,0,1)} = 0.0 % scalar measure for the exploitation of the trust region
        estAbsErr interval = interval.empty(1) % interval representing an estimate of the set of abstraction errors
        solver_time (1,1) double {mustBeNonnegative} = 0.0 % time required for solving the optimization problem
        opt_setupTime (1,1) double {mustBeNonnegative} = 0.0 % time required for setting up the optimization problem
    end
    
    properties(Dependent)
        costSetVolume 
    end

    methods
        function obj = termRegNonlinSysSolutionAuxData(flagIsFeasible,costSet,tr_exploit,estAbsErr,solver_time,opt_setupTime)
            %TERMREGNONLINSYSSOLUTIONDATACLASS Construct an instance of this class

            % set properties
            obj.flagIsFeasible = flagIsFeasible;
            obj.trustRegionExploitation = tr_exploit;
            obj.solver_time = solver_time;
            obj.opt_setupTime = opt_setupTime;
            obj.costSet = costSet;

            % check input arguments
            if isa(estAbsErr,'interval') && all(center(estAbsErr) == 0)
                obj.estAbsErr = estAbsErr;
            else
                error(['Estimate of the set of abstraction errors must be ' ...
                    'represented as an interval that is centered at the origin!'])
            end
        end

        function vol = get.costSetVolume(obj)
            % returns the volume of the set plugged into the cost function
            vol = volume(obj.costSet);
        end
    end
end
