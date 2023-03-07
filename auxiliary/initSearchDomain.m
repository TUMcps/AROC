function Tdomain = initSearchDomain(K,xEq,uEq,U,varargin)
% INITSEARCHDOMAIN - initialize search domain for terminal region
%
% Syntax:
%       Tdomain = INITSEARCHDOMAIN(K,xEq,uEq,U)
%       Tdomain = INITSEARCHDOMAIN(K,xEq,uEq,U,X)
%
% Description:
%       Initialize the search domain for determining a terminal region
%       using a heuristic that is based on the state and input constraints.
%
% Input Arguments:
%
%       -K:     feedback matrix for the controller (dimension: [nu,nx])
%       -xEq:   equilibrium point of the system (dimension: [nx,1])
%       -uEq:   control input for the equilibrium point (dimension: [nu,1])
%       -U:     set of control inputs (class: interval or zonotope)
%       -X:     set of state constraints (class: mptPolytope)
%
% Output Arguments:
%
%       -Tdomain:   search domain for the terminal region
%
% See Also:
%       computeTermRegZonoLinSys, computeTermRegSubpaving
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

    % parse input arguments
    X = [];
    
    if nargin > 4 && ~isempty(varargin{1})
       X.set = varargin{1}; 
    end
    
    % approximate the region where the input constraints are satisfied
    nx = size(K,2);
    U = interval(U) - uEq;
    len = sqrt(sum(K.^2,2))';
    width = min(abs([len'.*infimum(U);len'.*supremum(U)]));

    Tdomain = width*interval(-ones(nx,1),ones(nx,1)) + xEq;
    
    % scale the region so that is satisfies the state constraints
    if ~isempty(X) 
        
        % get properties from domain
        nx = dim(Tdomain);
        Z = zonotope(Tdomain);
        c = center(Z);
        G = generators(Z);
        
        % maximize volume of the set while satisfying the state constraints
        options = optimoptions('fmincon','display','off');
        
        obj = @(x) -prod(x(1:nx));
        
        C = abs(X.set.P.A*G);
        d = X.set.P.b - X.set.P.A*c;
        
        x = fmincon(obj,ones(nx,1),C,d,[],[],zeros(nx,1),ones(nx,1), ...
                                                              [],options);
        
        % construct final domain
        Tdomain = interval(zonotope(c,G*diag(x)));
    end
end