function poly = innerApproxPoly(Z)
% INNERAPPROXPOLY - inner-approximate a zonotope with a polytope
%
% Syntax:
%       poly = INNERAPPROXPOLY(Z)
%
% Description:
%       This function inner-approximates a zonotope with a polytope in 
%       halfspace representation that has less than 100 halfspaces.
%
% Input Arguments:
%
%       -Z:      zonotope object (class: zonotope)
%
% Output Arguments:
%
%       -poly:   inner-approximating polytope (class: mptPolytope)
%
% See Also:
%       linSysMPC, reachsetMPC
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % check if exact conversion is feasible
    n = dim(Z); m = size(Z.Z,2)-1;

    w = warning();
    warning('off');
    tmp = nchoosek(m,n-1);
    warning(w);

    if tmp < 100
        poly = mptPolytope(Z);
        return;
    end

    % compute tight inner-approximation by a simplex
    poly = innerApproxSimplex(Z);
end

function poly = innerApproxSimplex(Z)
% inner-approximation with a simplex

    % construct standard simplex
    n = dim(Z);
    V = eye(n+1);
    B = gramSchmidt(ones(n+1,1));
    dir = B(:,2:end)'*V;

    % determine boundary points in the direction of the simplex
    V = zeros(n,n+1);
    Z_ = Z + (-center(Z));

    for i = 1:n+1
        [~,V(:,i)] = supportFunc(Z_,dir(:,i));
    end

    V = V + center(Z);
    poly = mptPolytope(V');

    % ensure that the center of the zonotope is in the polytope
    if ~contains(poly,center(Z))

        % determine point in simplex closest to the center
        options = optimoptions('quadprog','display','off');

        x = quadprog(eye(n),-2*center(Z),poly.P.A,poly.P.b,[],[],[], ...
                                                            [],[],options);

        dir = center(Z) - x;
        
        % determine boundary point in the direction
        len = 2*radius(interval(Z));
        tmp = conZonotope(zeros(n,1),dir./norm(dir)*len);
        [~,v] = supportFunc(conZonotope(Z_) & tmp, dir);

        poly = mptPolytope([V, v + center(Z)]');
        poly = removeRedundancies(poly,'all');
    end
end