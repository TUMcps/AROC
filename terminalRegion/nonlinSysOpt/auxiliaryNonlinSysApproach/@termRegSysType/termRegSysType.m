classdef termRegSysType
    %TERMREGSYSTYPE - enumeration class for types of dynamical systems 
    %       considered in @termRegNonlinSysOpt algorithms.
    %
    % See Also:
    %       -
    % 
    % References:
    %       -
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

    enumeration
        SD % sampled-data system - continuous-time dynamics
        DT % discrete-time system
        paramDT % discrete-time system with uncertain parameters
    end

    methods
        function tf = isDiscreteTimeSystem(obj)
            tf = ismember(obj,[termRegSysType.DT,termRegSysType.paramDT]);
        end

        function tf = isParamSystem(obj)
            tf = ismember(obj,[termRegSysType.paramDT]);
        end

        function [res,errMsg] = isConsistentDiscretizationScheme(obj,integrator)
            assert(isa(integrator,'discretizeDynamics'))
            if obj.isDiscreteTimeSystem && xor( ...
                    obj.isParamSystem, ...
                    integrator.isParamSysDiscretizationScheme)
                    errMsg = ['System type (uncertain vs known parameters)' ...
                        'and time-discretization scheme are not consistent'];
                    res = false;
            else
                res = true;
                errMsg = '';
            end
        end 

        function integrator = getDefaultDiscretizationScheme(obj)
            if obj.isParamSystem
                integrator = discretizeDynamics.RK2param;
            else
                integrator = discretizeDynamics.RK2;
            end
        end
    end
end

