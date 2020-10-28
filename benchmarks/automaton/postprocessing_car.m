function R = postprocessing_car(R)
% POSTPROCESSING_CAR - compute the occupancy set from the reachable set
%
% Syntax:
%       R = POSTPROCESSING_CAR(R)
%
% Description:
%       This function computes the set occupied by the car form the 
%       computed reachable set. The occupancy set is later on used for 
%       collision checking during online application of the Manuever 
%       Automaton. For the car benchmark, the reachable set is projected to 
%       the position states and bloated by the size of the car.
%
% Input Arguments:
%
%       -R:     cell-array storing the reachable set
%
% Output Arguments:
%
%       -R:     cell-array storing the transformed reachable set
%
% See Also:
%       car, maneuverAuotomaton
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

    % car length and width in [m]
    l = 4.298;
    w = 1.674;
    
    car = zonotope(interval([-l/2;-w/2],[l/2;w/2]));
    
    % define transformation function
    syms x1 x2 phi z1 z2
    x = [x1;x2;phi;z1;z2];
    
    f = [x1 + cos(phi)*z1 - sin(phi)*z2; x2 + cos(phi)*z2 + sin(phi)*z1];
    
    % compute derivatives
    [fun,Afun,Qfun,Tfun] = computeDerivatives(f,x);    
    
    % loop over all reachable sets
    for i = 1:length(R)
    
        % reduce the zonotope order
        set = reduce(R{i}.set,'girard',3);
        
        % define initial set
        set = project(set,[3 4 2]);
    
        X = cartProd(set,car);
        
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

        R{i}.set = polygon(V(1,:)',V(2,:)');

        warning(w);
    end
    
    % unite occopancy sets to speed-up collision checking
    dT = 0.05;
    dT_ = 2*rad(R{1}.time);
    N = supremum(R{end}.time)/dT;
    N_ = dT / dT_;
    R_ = cell(N,1);
    counter = 1;
    
    for i = 1:N
       
       tStart = infimum(R{counter}.time);
       set = R{counter}.set;
       
       for j = 1:N_-1
          set = set | R{counter+j}.set; 
       end
        
       R_{i}.set = set;
       R_{i}.time = interval(tStart,tStart+dT);
       counter = counter + N_;
    end
    
    R = R_;
end

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


% Auxiliary Functions -----------------------------------------------------

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