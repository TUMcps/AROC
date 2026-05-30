function f = test_artificialSystem(x,u,w)
%TEST_ARTIFICIALSYSTEM - artificial system used in the unit
%test function of @computeTermRegNonlinSysLinApproach/checkInvariance.
%
% Syntax:
%       f = test_artificialSystem(x,u,w)
%
% Description:
%       Dynamics function of the artificial system used in the unit
%       test function of 
%       @computeTermRegNonlinSysLinApproach/checkInvariance. 
%
% Input Arguments:
%       -x:     system states x (dimension: [nx,1])
%       -u:     control inputs u (dimension: [nu,1]) 
%       -w:     disturbances w (dimension: [nw,1])
%
% Output Arguments:
%       -f:     value of the dynamic function = time derivative of x
%
% See Also:
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
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

f(1,1) =  u(1) + w(1) + 0.0001*x(1)^2;
f(2,1) = u(2) + w(2); 

end
