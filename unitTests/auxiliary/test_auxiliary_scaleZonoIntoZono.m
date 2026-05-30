function res = test_auxiliary_scaleZonoIntoZono
%TEST_AUXILIARY_SCALEZONOINTOZONO - unit test function of scaleZonoIntoZono
%
% Syntax:
%       res = test_auxiliary_scaleZonoIntoZono
%
% Description:
%       We generate random pairs of zonotopes, execute the function, and
%       check containment for the resulting inbody. To verify that there is
%       no larger scaling factor, we check whether a slightly bloated
%       inbody is still contained in the circumbody or not (expected 
%       result).
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       scaleZonoIntoZono, zonotope.generateRandom
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

% parameters for generating zonotopes
nx = 3;
orderIn = 4;
orderOut = 3;

nTestCases = 10;

% random testing - both sets centered at zero
res0center = false(nTestCases,1);
for ii = 1:nTestCases
    Zin = zonotope.generateRandom('Dimension',nx,'Center',zeros(nx,1), ...
        'NrGenerators',orderIn*nx);
    Zout = zonotope.generateRandom('Dimension',nx,'Center',zeros(nx,1), ...
        'NrGenerators',orderOut*nx);    
    scale = scaleZonoIntoZono(Zin,Zout);
    Zin_sc = zonotope(Zin.c,Zin.G*scale);
    Zin_sc_bloat = zonotope(Zin.c,Zin.G*scale*(1+1e-3));
    res0center(ii) = contains(Zout,Zin_sc,'approx:st',1e-6) ...
        && ~contains(Zout,Zin_sc_bloat,'approx:st',1e-6');
end

% random testing - random center, bloat circumbody to enable containment
% (test might still fail, see scaling of center in zonotope.generateRandom)
resRandCenter = false(nTestCases,1);
for ii = 1:nTestCases
    Zin = zonotope.generateRandom('Dimension',nx, ...
        'NrGenerators',orderIn*nx);
    Zout = zonotope.generateRandom('Dimension',nx, ...
        'NrGenerators',orderOut*nx);   
    Zout = zonotope(Zout.c,Zout.G*15);
    % solver might fail since only the generator matrix is scaled
    try
        scale = scaleZonoIntoZono(Zin,Zout);
        Zin_sc = zonotope(Zin.c,Zin.G*scale);
        Zin_sc_bloat = zonotope(Zin.c,Zin.G*scale*(1+1e-3));
        resRandCenter(ii) = contains(Zout,Zin_sc,'approx:st',1e-6) ...
            && ~contains(Zout,Zin_sc_bloat,'approx:st',1e-6');
    catch
        resRandCenter(ii) = true;
    end
end

res = all(res0center) && all(resRandCenter);

end
