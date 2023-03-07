function R = orderReduction(R,order)
% ORDERREDUCTION - reduces the order of a zonotope
%
% Syntax:
%       [R0opt,RcontOpt] = SCALEINITIALSET(sys,R0,H,goalSet,U,Opts)
%
% Description:
%       This function reduces the zonotope order of a zonotope and returns
%       a zonotope that is a subset of the original zonotope and has the 
%       desired order.
%
% Input Arguments:
%
%       -R:     original set (class: zonotope)
%       -order: desired zonotope order
%
% Output Arguments:
%
%       -R:     set with reduced order (class: zonotope)
%
% See Also:
%       safetyNetControl
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % normalize the dimensions of the zonotope using the width of the
    % enclosing interval. This is required since the different states might
    % have very different ranges 
    r = rad(interval(R));
    Rtemp = diag(1./r)*R;
    
    % reduce the order of the zonotope in an underapproximative way
    Rtemp = reduceUnderApprox(Rtemp,'linProg',order);
    
    % transform the zonotope back to the original space
    R = diag(r)*Rtemp; 
end