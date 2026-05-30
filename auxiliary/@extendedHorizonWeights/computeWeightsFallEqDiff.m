function w = computeWeightsFallEqDiff(M)
%COMPUTEWEIGHTSFALLEQDIFF- weighting function for an extended optimization
%       horizon - 'fallEqDiff'.
%
% Syntax:
%       w = computeWeightsFallEqDiff(M)
%
% Description:
%       This function returns the values of the weighting function
%       'fallEqDiff'.
%
% Input Arguments:
%       -M:     extended optimization horizon
%               (positive integer)
%
% Output Arguments:
%       -w:     values of the weighting function
%               (double array of dimension [M,1])
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

w = zeros(M,1);
w(M) = 1;
for ii = M-1:-1:1
   w(ii) = sum(w(ii+1:M)); 
end
w = w/sum(w(2:M)); 

end
