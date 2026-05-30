function res = test_computeTermRegNonlinSysPolyApproach_paramPartZonotope()
%TEST_COMPUTETERMREGNONLINSYSPOLYAPPROACH_PARAMPARTZONOTOPE - unit test 
% function of paramPartZonotope
%
% Syntax:
%       res = test_computeTermRegNonlinSysPolyApproach_paramPartZonotope
%
% Description:
%       Unit test function of the parameterized partial zonotope
%       over-approximation of a given polynomial zonotope.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/paramPartZonotope
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
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% setup polynomial zonotope
pZ = polyZonotope([4;4], [2, 1, 2, 0.5; 0, 2, 2, 1], [], [ 2, 1, 0, 1; 2, 0, 3, 0; 0, 1, 0 2; 0, 1, 0, 1], [1;2;-1;-2]);

% ids for zonotope over-approximation
idv = [1; 2];

% ids of scalar variables
idp = [-1; -2];

% compute partial zonotope over-approximation
partZono = computeTermRegNonlinSysPolyApproach.paramPartZonotope(pZ,idv,idp);

% re-assemble
idp_val = 5*rand(2,1) + 2;
c = resolve(partZono.c,idp_val,idp);
% ... collect the generators of the partial zonotope
G = [];
for ii = 1:length(partZono.G)
    G = [G, resolve(partZono.G{ii},idp_val,idp)]; 
end

% validate the result
res = contains(zonotope(c,G), zonotope(resolve(pZ,idp_val,idp)));

end