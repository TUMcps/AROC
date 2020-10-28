function [res,A,B,D,c] = isLinearModel(model,nx,nu,nw)
% ISLINEARMODEL - check if a model is linear or nonlinear
%
% Syntax:
%       [res,A,B,D,c] = ISLINEARMODEL(model,nx,nu,nw)
%
% Description:
%       Checks if a model is linear of nonlinear. If the model is linear,
%       the function returns the system matrices of the linear system
%       equation
%
%       $\dot x = A \cdot x + B \cdot u + D \cdot w + c$
%
% Input Arguments:
%
%       -model: function handle to the dynamic function of the model
%       -nx:    number of system states
%       -nu:    number of inputs
%       -nw:    number of disturbances
%
% Output Arguments:
%
%       -res:   1 if the model is linear, 0 if not
%       -A:     system matrix of the linear model (dimension: [nx,nx])
%       -B:     input matrix of the linear model (dimension: [nx,nu])
%       -D:     disturbance matrix of the linear model (dimension: [nu,nw])
%       -c:     constand offset of the linear model (dimension: [nx,1])
%
% See Also:
%       platoon, car
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

    res = 1;
    A = []; B = []; D = []; c = [];

    % create symbolic variables
    x = sym('x',[nx,1]);
    u = sym('u',[nu,1]);
    w = sym('w',[nw,1]);
    
    % insert the symbolic vectors into the dynamic equation
    f = model(x,u,w);
    func = matlabFunction(f,'Vars',{x,u,w});
    
    % check if the model is linear and return the system matrices if it is
    eqns = [f==ones(nx,1)];
    
    try
        % extract system matrices
        A_ = equationsToMatrix(eqns,x);
        B_ = equationsToMatrix(eqns,u);
        D_ = equationsToMatrix(eqns,w);
        c_ = func(zeros(nx,1),zeros(nu,1),zeros(nw,1));

        % convert from symbolic to numerical
        A = double(A_);
        B = double(B_);
        D = double(D_);
        c = double(c_);
    catch
        res = 0;   
    end   
end
