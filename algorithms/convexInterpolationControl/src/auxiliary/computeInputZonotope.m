function [c,G] = computeInputZonotope(uCorner,uCenter,zonoR,Opts)
% COMPUTEINPUTZONOTOPE - Compute a zonotope approximating the inputs
%
% Syntax:
%       [c,G] = COMPUTEINPUTZONOTOPE(uCorner,uCenter,zonoR,Opts)
%
% Description:
%       This function computes a zonotope for the input values. This
%       zonotope corresponds the linear control law evaluated on the state 
%       zonotope (see Sec. 5 in [1]).
%
% Input Arguments:
%
%       -uCorner:       optimal control inputs for the corner trajectories
%                       (dimension: [nu,2^nx])
%       -uCenter:       optimal control inputs for the center trajectory
%                       (dimension: [nx, Opts.Nc])
%       -zonoR:         parallelotope which is an overapproximation of the
%                       initial zonotope zono (class zonotope)
%       -Opts:          structure containing user defined options for the 
%                       algorithm 
%
% Output Arguments:
%       -c:         center of the input zonotope 
%       -G:         generator matrix of the input zonotope
%
% See Also:
%       convexInterpolationControl, computeLinearController
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


    %% Initialization

    % extract required data from Opts-struct
    nx = Opts.nx;
    nu = Opts.nu;
    I = Opts.I;
    u_max = Opts.uMax;
    u_min = Opts.uMin;

    % compute extended matrix containing the alpha values of the extreme 
    % points
    Itemp = [ones(2^nx,1),I'];
    I_big = kron(Itemp,eye(nu));

    % save the inputs for all corner points in the same vector
    u_x_temp = [];        
    for i = 1:1:2^nx
        u_x_temp = [u_x_temp; uCorner(:,i)];
    end


    %% Optimization 

    if strcmp(Opts.approx.method,'optimized')

        [c,G]   = inputZonotopeOptimized(uCorner,zonoR,I_big,u_max,u_min,u_x_temp,nx,nu,Opts);

    elseif strcmp(Opts.approx.method,'center') 

        [c,G] = inputZonotopeCenter(uCenter,I_big,u_max,u_min,u_x_temp,nx,nu);

    elseif strcmp(Opts.approx.method,'scaled') 

        [c,G] = inputZonotopeScaled(I_big,u_x_temp,u_max,u_min,nx,nu);

    else
        error('Wrong value for "Opts.approx.method"!');       
    end
end

    
%% Auxiliary functions
    
function [c,G] = inputZonotopeOptimized(uCorner,zonoR,I_big,u_max,u_min,u_x_temp,nx,nu,Opts)

    %% Part 1: Good Approximation fo the linearized exact control law
        
    % compute symbolic function and jacobian
    x = sym('x', [nx 1]);
    funcSymb = exactControlLaw(x,zonoR,uCorner(:,:,1));
    jacoSymb = jacobian(funcSymb,x);

    % evaluate function and jacobian at center trajectory
    cr = center(zonoR);
    Gr = generators(zonoR);

    funcSymb = subs(funcSymb, x, cr);
    func = eval(funcSymb);

    jacoSymb = subs(jacoSymb, x, cr);
    jaco = eval(jacoSymb);

    % convert linearized system to vector for the quadratic program
    temp = reshape(jaco,[nx*nu,1]);
    xLin = [func;temp];

    % compute matrizes for the quadratic program
    Gr_inv = inv(Gr);

    Atemp = zeros(nx*nu);
    index = 1;
    for i = 1:nx
        for j = 1:nu
           eTemp = zeros(1,nu);
           eTemp(j) = 1;

           test = kron(Gr_inv(:,i)',eTemp);
           Atemp(index,:) = test;
           index = index + 1;
        end
    end

    Atemp = [eye(nu) zeros(nu,nx*nu);zeros(nx*nu,nu) Atemp];

    A_exact = 2*(Atemp'*Atemp);
    B_exact = -2*xLin'*Atemp;


    %% Part 2: Good Approximation of the corner inputs

    % create matrices of the quadratic program: f = 0.5*x' A x + B' x
    A = zeros(size(I_big,2));
    B = zeros(1,size(I_big,2));
    errorRemainder = 0;

    index = 1;
    for i = 1:2^nx
       Isample = I_big(index:index+nu-1,:);
       Usample = u_x_temp(index:index+nu-1);
       index = index + nu;

       A = A + 2*(Isample'*Isample);
       B = B - 2*Usample'*Isample;
       errorRemainder = errorRemainder + Usample'*Usample;  
    end

    % create matrices for the constraints: C x < d

    C = zeros(2*size(I_big,1),size(I_big,2));
    d = zeros(size(C,1),1);

    index1 = 1;
    index2 = 1;
    for i = 1:2^nx
        Isample = I_big(index1:index1+nu-1,:);

        C(index2:index2+nu-1,:) = Isample;
        d(index2:index2+nu-1,:) = u_max;

        index2 = index2+nu;

        C(index2:index2+nu-1,:) = -Isample;
        d(index2:index2+nu-1,:) = -u_min;   

        index1 = index1 + nu;
        index2 = index2 + nu;
    end


    %% Quadratic Program

    % add term that penalizes the differnence to the linearized system
    lambda = Opts.approx.lambda;

    A_ = (1-lambda) * A + lambda * A_exact;
    B_ = (1-lambda) * B + lambda * B_exact;

    % solve quadratic program with constraints
    options = optimoptions('quadprog','Display','off');
    x_exact = quadprog(A_,B_',C,d,[],[],[],[],[],options);
    un_exact = reshape(x_exact,[nu,nx+1]);
    c = un_exact(:,1);
    G = un_exact(:,2:end);
end
    
function [c,G] = inputZonotopeCenter(uCenter,I_big,u_max,u_min,u_x_temp,nx,nu)

    % create matrices of the quadratic program: f = 0.5*x' A x + B' x
    Ac = zeros(size(I_big,2)-nu);
    Bc = zeros(1,size(I_big,2)-nu);
    errorRemainder_cent = 0;

    index = 1;
    for i = 1:2^nx
       Isample = I_big(index:index+nu-1,nu+1:end);
       Usample = u_x_temp(index:index+nu-1)-uCenter;
       index = index + nu;

       Ac = Ac + 2*(Isample'*Isample);
       Bc = Bc - 2*Usample'*Isample;
       errorRemainder_cent = errorRemainder_cent + Usample'*Usample;  
    end

    % create matrices for the constraints: C x < d
    Cc = zeros(2*size(I_big,1),size(I_big,2)-nu);
    dc = zeros(size(Cc,1),1);

    index1 = 1;
    index2 = 1;
    for i = 1:2^nx
        Isample = I_big(index1:index1+nu-1,nu+1:end);

        Cc(index2:index2+nu-1,:) = Isample;
        dc(index2:index2+nu-1,:) = u_max-uCenter;

        index2 = index2+nu;

        Cc(index2:index2+nu-1,:) = -Isample;
        dc(index2:index2+nu-1,:) = -(u_min-uCenter);   

        index1 = index1 + nu;
        index2 = index2 + nu;
    end

    % solve quadratic program with constraints
    options = optimoptions('quadprog','Display','off');
    x_cent = quadprog(Ac,Bc',Cc,dc,[],[],[],[],[],options);
    G = reshape(x_cent,[nu,nx]);
    c = uCenter;
end
    
function [c,G] = inputZonotopeScaled(I_big,u_x_temp,u_max,u_min,nx,nu)

    % solve least square optimization problem
    u_new=I_big\u_x_temp;

    un = reshape(u_new,[nu,nx+1]);
    
    % center the input set around the origin
    U = interval(u_min,u_max);
    mu = center(U);
    uMax = supremum(U-mu);
    
    un(:,1) = un(:,1) - mu;

    % scale input zonotope to satisfy the constraints
    constraint_test = abs(un)*ones(nx+1,1)-uMax;
    for i=1:nu
        if constraint_test(i)>0
            un(i,:)=u_max(i)/(u_max(i)+constraint_test(i))*un(i,:);
        end
    end

    % obtain the center and the generator matrix of the input zonotope
    c = un(:,1)+mu;
    G = un(:,2:end);
end