function decay = decayFunctions(index,Opts)
% DECAYFUNCTIONS - compute decay functions for optimal control problems
%                  with extended optimization horizon
%
% Syntax:
%       decay = DECAYFUNCTIONS(index,Opts)
%
% Description:
%       This function solves an optimal control problem for all corner
%       states of the initial parallelotope.
%
% Input Arguments:
%
%       -index:         lenght of the remaining optimizaion horizon
%       -Opts:          structure containing user defined options for the 
%                       algorithm  
%
% Output Arguments:
%
%       -decay:     vector contaning the weights for all points of the
%                   remaining optimization horizon
%                   (dimension: [index,1])
%
% See Also:
%       convexInterpolationControl, writeAcadoFiles
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------


    % select weight function
    
    if strcmp(Opts.extHorizon.decay,'uniform')            % uniform
        
        decay = ones(index,1);
     
    elseif strcmp(Opts.extHorizon.decay,'fall') || ...
           strcmp(Opts.extHorizon.decay,'fall+End')       % fall
        
        decay = zeros(index,1);
        for i = 1:index
           decay(i) = 1/i; 
        end
        decay = decay/max(decay);
        
    elseif strcmp(Opts.extHorizon.decay,'rise')            % rise
        
        decay = zeros(index,1);
        for i = 1:index
           decay(i) = 1/i; 
        end
        decay = fliplr(decay');
        decay = decay'/max(decay);
        
    elseif strcmp(Opts.extHorizon.decay,'quad')           % quad
        
        decay = zeros(index,1);
        middle = (index+1)/2;
        for i = 1:index
            decay(i) = floor(abs(i-middle))^2 + 1;
        end
        decay = decay/max(decay);
        
    elseif strcmp(Opts.extHorizon.decay,'fallLinear') || ...
           strcmp(Opts.extHorizon.decay,'fallLinear+End') % fall linear
        
        decay = zeros(index,1);
        decay(1) = 1;
        for i = 2:index
            decay(i) = 1-(i-1)*(1-1/index)/(index-1); 
        end
        
    elseif strcmp(Opts.extHorizon.decay,'fallEqDiff') || ...
           strcmp(Opts.extHorizon.decay,'fallEqDiff+End') % fall equal difference
        
        decay = zeros(index,1);
        decay(1) = 1;
        for i = 2:index
           decay(i) = sum(decay(1:i-1)); 
        end
        decay = fliplr(decay');
        decay = decay'/max(decay); 
        
    elseif strcmp(Opts.extHorizon.decay,'riseLinear')     % rise linear
        
        decay = zeros(index,1);
        decay(1) = 1;
        for i = 2:index
            decay(i) = 1-(i-1)*(1-1/index)/(index-1); 
        end
        decay = fliplr(decay')'; 
         
    elseif strcmp(Opts.extHorizon.decay,'riseEqDiff')     % rise equal difference    
        
        decay = zeros(index,1);
        decay(1) = 1;
        for i = 2:index
           decay(i) = sum(decay(1:i-1)); 
        end
        decay = decay/max(decay);  
        
    elseif strcmp(Opts.extHorizon.decay,'end')            % end only
        
        decay = zeros(index,1);
        decay(index) = 1;
        
    else
        
        error('Wrong value for "Opts.extHorizon.decay"');
        
    end
    
    
    % additionaly weight the end point
    if ismember(Opts.extHorizon.decay,{'fall+End','fallLinear+End','fallEqDiff+End'})
       decay(index) = 1; 
    end