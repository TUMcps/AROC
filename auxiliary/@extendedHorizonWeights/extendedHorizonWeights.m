classdef extendedHorizonWeights
    %EXTENDEDHORIZONWEIGHTS - enumeration class for extended horizon cost
    %       function weights.
    % 
    % Description:
    %       Enumeration class implementing the different types of weighting
    %       functions used for the extended optimization horizon in some of
    %       the motion primitive-based control algorithms. See also Sec.
    %       7.5 in the manual.
    %
    % See Also:
    %       generatorSpaceControl, convexInterpolationControl,
    %       polynomialControl
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
    % Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
    %               Embedded Systems, TU Muenchen
    %------------------------------------------------------------------
    
    enumeration
        uniform 
        final
        fall
        rise
        riseLinear
        riseEqDiff
        quad
        fallFinal
        fallLinear
        fallLinearFinal
        fallEqDiff
        fallEqDiffFinal
    end
    
    methods
       
        function w = computeWeights(obj,M)
            %COMPUTEWEIGHTS Summary of this method goes here
            %   Detailed explanation goes here
            %
            % Input Arguments:
            %       -obj:   instance of enumeration class 
            %               extendedHorizonWeights
            %       -M:     extended optimization horizon
            %               (positive integer)
            %
            % Output Arguments:
            %       -w:     vector storing the values of the weighting 
            %               function at the next M time steps
            %               (array of dimension [M,1])
            %
            % ------------------------------------------------------------

            % check input arguments
            if ~(~isinf(M) && floor(M) == M)
                error(['Extended optimization horizon must be a ' ...
                    'positive integer.'])
            end

            if M == 1
                w = 1;
            else
                if obj == extendedHorizonWeights.uniform
                    w = ones(M,1);
                elseif obj == extendedHorizonWeights.final
                    w = zeros(M,1);
                    w(M) = 1;
                elseif obj == extendedHorizonWeights.fall
                    w = obj.computeWeightsFall(M);
                elseif obj == extendedHorizonWeights.rise
                    w = obj.computeWeightsRise(M);
                elseif obj == extendedHorizonWeights.riseLinear
                    w = obj.computeWeightsRiseLinear(M);
                elseif obj == extendedHorizonWeights.riseEqDiff
                    w = obj.computeWeightsRiseEqDiff(M);
                elseif obj == extendedHorizonWeights.quad
                    w = obj.computeWeightsQuad(M);
                elseif obj == extendedHorizonWeights.fallFinal
                    w = obj.computeWeightsFallFinal(M);
                elseif obj == extendedHorizonWeights.fallLinear
                    w = obj.computeWeightsFallLinear(M);
                elseif obj == extendedHorizonWeights.fallLinearFinal
                    w = obj.computeWeightsFallLinearFinal(M);
                elseif obj == extendedHorizonWeights.fallEqDiff
                    w = obj.computeWeightsFallEqDiff(M);
                elseif obj == extendedHorizonWeights.fallEqDiffFinal
                    w = obj.computeWeightsFallEqDiffFinal(M);
                else
                    error(['No weights computation function for given ' ...
                        'instance of extendedHorizonWeights implemented!'])
                end
            end
        end
    end

    methods (Static)
        % fall
        w = computeWeightsFall(M)

        % rise
        w = computeWeightsRise(M)

        % quad
        w = computeWeightsQuad(M)

        % fallLinear
        w = computeWeightsFallLinear(M)

        % fallEqDiff
        w = computeWeightsFallEqDiff(M);
    end
end
