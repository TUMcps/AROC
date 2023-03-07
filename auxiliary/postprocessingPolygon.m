function O = postprocessingPolygon(f,P,R,varargin)
% POSTPROCESSINGPOLYGON - compute the occupancy set from the reachable set
%
% Syntax:
%       O = POSTPROCESSINGPOLYGON(f,P,R)
%       O = POSTPROCESSINGPOLYGON(f,P,R,dt,order,tol)
%
% Description:
%       This function computes the occupancy set from the reachable set for
%       the case where the occupancy set is 2-dimensional and can therefore 
%       be represented by a polygon.
%
% Input Arguments:
%
%       -f:     function handle to the function f(x,p) that computes the
%               occupancy set from the states x using the additional 
%               parameters p 
%       -R:     cell-array storing the reachable set
%       -P:     set representing the possible values for the parameters p
%               that are passed to the fucntion f(x,p)
%       -dt:    time step size for the resulting occupancy set
%       -order: zonotope order to which the reachable set is reduced before
%               the occupancy set is calculated
%       -tol:   tolerance for occupancy set computation
%
% Output Arguments:
%
%       -O:     cell-array storing the occupancy set
%
% See Also:
%       maneuverAutomaton
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
    
    % parse input arguments
    dt = 2*rad(R{1}.time); order = 3; tol = [];

    if nargin > 3 && ~isempty(varargin{1})
        dt = varargin{1};
    end
    
    if nargin > 4 && ~isempty(varargin{2})
        order = varargin{2};
    end

    if nargin > 5 && ~isempty(varargin{3})
        tol = varargin{3};
    end

    % check user inputs
    if dt < 2*rad(R{1}.time)
        error('Time step size "dt" smaller than reachability time step.');
    end

    % get transformation function in symbolic form
    x = sym('x',[dim(R{1}.set),1]);
    p = sym('p',[dim(P),1]);

    fsym = f(x,p);

    % determine states that appear in the function
    tmp = (1:length(x));
    ind = tmp(ismember(x,symvar(fsym)));
    
    % compute derivatives
    [fun,Afun,Qfun,Tfun] = computeDerivatives(fsym,[x(ind);p]);    
    
    % loop over all reachable sets
    for i = 1:length(R)
    
        % reduce the zonotope order
        set = reduce(R{i}.set,'girard',order);
        
        % define initial set
        X = cartProd(project(set,ind),P);
        
        % evaluate derivatives at linearization point
        p = center(X);

        [f,A,Q,T] = evalDerivatives(X,p,fun,Afun,Qfun,Tfun);

        % compute Largrange remainder
        rem = lagrangeRemainder(X,p,T);

        % compute over-approximating zonotope
        res = f + A * (X + (-p)) + 0.5*quadMap((X + (-p)),Q) + rem;

        % convert to polygon object
        if ~isa(res,'zonotope')
           res = zonotope(res); 
        end

        V = vertices(res);

        w = warning();
        warning('off');

        R{i}.set = polygon(V(1,:),V(2,:));

        warning(w);
    end
    
    % unite occopancy sets to speed-up collision checking
    N = round(supremum(R{end}.time)/dt);
    t = linspace(infimum(R{1}.time),supremum(R{end}.time),N+1);
    O = cell(N,1);
    counter = 2;
    set = R{1}.set;
    
    for i = 1:N
       
       while counter <= length(R) && infimum(R{counter}.time) < t(i+1)-eps
          set = set | R{counter}.set; 
          counter = counter + 1;
       end
       
       if ~isempty(tol)
          O{i}.set = simplify(set,tol);
       else
          O{i}.set = set;
       end

       O{i}.time = interval(t(i),t(i+1));

       if supremum(R{counter-1}.time) > t(i+1) + eps
           set = R{counter-1}.set;
           counter = counter + 1;
       else
           set = [];
       end
    end
end


% Auxiliary Functions -----------------------------------------------------

function [fun,Afun,Qfun,Tfun] = computeDerivatives(f,x)
% compute the symbolic derivatives of the function

    % function handle for the nonlinear function
    fun =  matlabFunction(f,'Vars',{x});
    
    A = jacobian(f,x);
    Afun =  matlabFunction(A,'Vars',{x});
    
    Qfun = cell(length(f),1);
    for i = 1:length(f)
       temp = hessian(f(i),x); 
       Qfun{i} =  matlabFunction(temp,'Vars',{x});
    end
    
    Tfun = cell(size(A));
    for i = 1:size(A,1)
        for j = 1:size(A,2)
            temp = hessian(A(i,j),x);
            Tfun{i,j} = matlabFunction(temp,'Vars',{x});
        end
    end
end

function [f,A,Q,T] = evalDerivatives(X,p,fun,Afun,Qfun,Tfun)
% evaluate the derivatives at the linearization point

    % interval encluore of the set
    int = interval(X);
    
    f = fun(p);
    A = Afun(p);
    
    Q = cell(length(f),1);
    for i = 1:length(f)
       funHan = Qfun{i};
       Q{i} = funHan(p);
    end
    
    T = cell(size(A));
    for i = 1:size(A,1)
        for j = 1:size(A,2)
            funHan = Tfun{i,j};
            T{i,j} = funHan(int);
        end
    end
end

function rem = lagrangeRemainder(X,p,T)
% comptute the lagrange remainder of the Taylor series

    % interval enclousre of the shifted initial set
    int = interval(X) - p;

    % Lagrange remainder term
    rem = interval(zeros(size(T,1),1));
    
    for i = 1:size(T,1)
        for j = 1:size(T,2)
            rem(i) = rem(i) + int(j) * transpose(int) * T{i,j} * int;
        end
    end
    
    % convert to zonotope
    rem = zonotope(1/6*rem);
end