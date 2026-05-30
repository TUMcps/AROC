function approxVol = cvx_approxZonoVolume(obj,s)
%CVX_APPROXZONOVOLUME - evaluates the concave approximation of
%       the volume of a zonotope in the generator scaling framework.
%
% Syntax:
%       approxVol = cvx_approxZonoVolume(obj,s)
%
% Description:
%       This function evaluates the concave approximation of the volume of
%       a zonotope in the generator scaling framework [1, Eq.30].
%
% Input Arguments:
%       -obj:       instance of class computeTermRegNonlinSysLinApproach
%       -s:         generator scaling factors for the cost set
%                   (positive double vector)
%   
% Output Arguments:
%       -approxVol:     value of the concave approximation of the zonotope
%                       volume
%                       (positive scalar)
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

costCoefficients_normalized = obj.costCoefficients/max(obj.costCoefficients);
approxVol = 0;
for ii = 1:size(obj.costExponents,1)
    approxVol = approxVol ...
        + costCoefficients_normalized(ii)*geomean(s(obj.costExponents(ii,:)));
end
   
end
