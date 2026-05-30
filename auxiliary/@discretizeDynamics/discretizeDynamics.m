classdef discretizeDynamics
    %DISCRETIZEDYNAMICS - enumeration class for (time-)discretization
    %       schemes of (nonlinear) dynamical systems.
    %
    % See Also:
    %       Rk2integrator, RK4integrator
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
        FWE % forward Euler method
        RK2 % explicit midpoint method
        RK4 % classical Runge-Kutta method

        % systems with uncertain parameters
        RK2param % explicit midpoint method for systems with uncertain parameters
        RK4param % classical Runge-Kutta method for systems with uncertain parameters
    end
    
    methods    
        function fH = getFunHandle(obj)
            %INTEGRATOR - returns a handle to the function implementing the
            %chosen integration scheme.
                
            if obj == discretizeDynamics.FWE
                fH = @FWEintegrator;
            elseif obj == discretizeDynamics.RK2
                fH = @RK2integrator;
            elseif obj == discretizeDynamics.RK4
                fH = @RK4integrator;
            elseif obj == discretizeDynamics.RK2param
                fH = @RK2paramIntegrator;
            elseif obj == discretizeDynamics.RK4param
                fH = @RK4paramIntegrator;
            end
        end

        function tf = isParamSysDiscretizationScheme(obj)
            tf = ismember(obj,[ ...
                discretizeDynamics.RK2param,...
                discretizeDynamics.RK4param ...
                ]);
        end
    end
end
