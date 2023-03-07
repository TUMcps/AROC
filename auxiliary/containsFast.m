function res = containsFast(set,points)
% containsFast - accelerated containment check for point clouds
%
% Syntax:
%       res = containsFast(set,points)
%
% Description:
%       This function checks if a point cloud is contained in a set. The
%       implementation provided in this function is much faster than the
%       implementation in CORA.
%
% Input Arguments:
%
%       -set:           CORA contSet object (class interval, mptPolytope,
%                       or zonotope)
%       -points:        point clound stored in a matrix (dimension [n,K])
%
% Output Arguments:
%
%       -res:       point cloud contained (res = true) or not (res = false)
%
% See Also:
%       results
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

    if isa(set,'zonotope')

        c = center(set);
        G = generators(set);
        m = size(G,2);
        f = ones(m,1); lb = -f; ub = f;
        options = optimoptions('linprog','display','off');
        res = true;

        for i = 1:size(points,2)

            [~,~,exitflag] = linprog(f,[],[],G,points(:,1)-c,lb,ub,options);

            if exitflag < 1
                res = false; return;
            end
        end

    else
        res = all(contains(set,points));
    end
end