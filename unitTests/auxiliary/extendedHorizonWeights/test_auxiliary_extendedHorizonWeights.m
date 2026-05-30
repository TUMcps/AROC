function res = test_auxiliary_extendedHorizonWeights()
%EXTENDEDHORIZONWEIGHTS - unit test function of the enumeration class
%extendedHorizonWeights.
%
% Syntax:
%       res = test_auxiliary_extendedHorizonWeights()
%
% Description:
%       Compares the correctness of the computed weights for a randomly 
%       chosen extended optimization horizon.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @extendedHorizonWeights
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

% single step
M = 1;
res1 = runWeightingFunctions(M);


% random length
resRand = true;
for ii = 1:10
    M = randi(100) + 1;
    resRand = resRand && all(runWeightingFunctions(M));
end

res = all(res1) && all(resRand);

end


%% auxiliary functions


function res = runWeightingFunctions(M)

    res = [];
     
    % uniform
    funType = extendedHorizonWeights.uniform;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % final
    funType = extendedHorizonWeights.final;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fall
    funType = extendedHorizonWeights.fall;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % rise
    funType = extendedHorizonWeights.rise;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % riseLinear
    funType = extendedHorizonWeights.riseLinear;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % riseEqDiff
    funType = extendedHorizonWeights.riseEqDiff;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % quad
    funType = extendedHorizonWeights.quad;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fallFinal
    funType = extendedHorizonWeights.fallFinal;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fallLinear
    funType = extendedHorizonWeights.fallLinear;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fallLinearFinal
    funType = extendedHorizonWeights.fallLinearFinal;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fallEqDiff
    funType = extendedHorizonWeights.fallEqDiff;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

    % fallEqDiffFinal
    funType = extendedHorizonWeights.fallEqDiffFinal;
    w = funType.computeWeights(M);
    w_exp = computeExpectedWeights(funType,M);
    res = [res, all(w == w_exp)];

end

function w = computeExpectedWeights(weightingFunctionType,M)

    if M==1
        w = 1;
        return;
    end

    % select weight function
    
    if weightingFunctionType == extendedHorizonWeights.uniform  
        % uniform
        
        w = ones(M,1);
     
    elseif weightingFunctionType == extendedHorizonWeights.fall || ...
           weightingFunctionType == extendedHorizonWeights.fallFinal    
        % fall
        
        w = zeros(M,1);
        for i = 1:M
           w(i) = 1/i; 
        end
        w = w/max(w);
        
    elseif weightingFunctionType == extendedHorizonWeights.rise
        % rise
        
        w = zeros(M,1);
        for i = 1:M
           w(i) = 1/i; 
        end
        w = fliplr(w')';
        w = w'/max(w);
        
    elseif weightingFunctionType == extendedHorizonWeights.quad
        % quad
        
        w = zeros(M,1);
        middle = (M+1)/2;
        for i = 1:M
            w(i) = floor(abs(i-middle))^2 + 1;
        end
        w = w/max(w);
        
    elseif weightingFunctionType == extendedHorizonWeights.fallLinear || ...
        weightingFunctionType == extendedHorizonWeights.fallLinearFinal
        % fall linear
        
        w = zeros(M,1);
        w(1) = 1;
        for i = 2:M
            w(i) = 1-(i-1)*(1-1/M)/(M-1); 
        end
        
    elseif weightingFunctionType == extendedHorizonWeights.fallEqDiff || ...
            weightingFunctionType == extendedHorizonWeights.fallEqDiffFinal
        % fall equal difference
        
        w = zeros(M,1);
        w(1) = 1;
        for i = 2:M
           w(i) = sum(w(1:i-1)); 
        end
        w = fliplr(w');
        w = w'/max(w); 
        
    elseif weightingFunctionType == extendedHorizonWeights.riseLinear
        % rise linear
        
        w = zeros(M,1);
        w(1) = 1;
        for i = 2:M
            w(i) = 1-(i-1)*(1-1/M)/(M-1); 
        end
        w = fliplr(w')'; 
         
    elseif weightingFunctionType == extendedHorizonWeights.riseEqDiff 
        % rise equal difference    
        
        w = zeros(M,1);
        w(1) = 1;
        for i = 2:M
           w(i) = sum(w(1:i-1)); 
        end
        w = w/max(w);  
        
    elseif weightingFunctionType == extendedHorizonWeights.final
        % final state only
        
        w = zeros(M,1);
        w(M) = 1;
        
    else
        
        error('Wrong value for "Opts.extHorizon.decay"');
        
    end
    
    
    % additionaly weight the final state 
    if ismember(weightingFunctionType, [extendedHorizonWeights.fallFinal, ...
            extendedHorizonWeights.fallLinearFinal, ...
            extendedHorizonWeights.fallEqDiffFinal])
       w(M) = 1; 
    end

end
