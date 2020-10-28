function vert = symExtremePoints(zonoR, I)
% SYMEXTREMEPOINTS - compute vertices of a parallelotope
%
% Syntax:
%       vert = SYMEXTREMEPOINTS(zonoR, I)
%
% Description:
%       Compute the vertices of a parallelotope using the specified alpha
%       values 
%
% Input Arguments:
%
%       -zonoR:         parallelotope (class: zonotope)
%       -I:             matrix containing the zonotope parameters alpha for
%                       all vertices (dimension: [nx, 2^nx])
%
% Output Arguments:
%
%       -vert:          vertices of the parallelotope "zonoR"
%                       (dimension: [nx,2^nx])
%
% See Also:
%       convexInterpolationControl
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

    Zm = zonoR.Z;
    n = size(Zm,1);

    Iextended=[ones(1,2^n);I];

    vert=zeros(n,2^n);
    for i=1:2^n
        vert(:,i)=Zm*Iextended(:,i);
    end
    