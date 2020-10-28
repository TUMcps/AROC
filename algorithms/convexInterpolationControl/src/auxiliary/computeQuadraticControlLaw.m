function params = computeQuadraticControlLaw(uCorner,zonoR,Opts)
% COMPUTEQUADRATICCONTROLLAW - Compute the parameter of the quadratic
%                              control law
%
% Syntax:
%       params = COMPUTEQUADRATICCONTROLLAW(uCorner,zonoR,Opts)
%
% Description:
%       This function computes a zonotope for the input values. This
%       zonotope is a linear approximation of the exact control law.
%
% Input Arguments:
%
%       -uCorner:       optimal control inputs for the corner trajectories
%                       (dimension: [nu,2^nx])
%       -zonoR:         parallelotope which is an overapproximation of the
%                       initial zonotope zono (class zonotope)
%       -Opts:          structure containing user defined options for the 
%                       algorithm        
%
% Output Arguments:
%
%       -params:        vector storing the parameter of the quadratic
%                       control law (dimension: [nu*(2*nx+1),1])
%
% See Also:
%       convexInterpolationControl, computeQuadraticController
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


    %% Extract Parameters

    % extract important parameters from the Opts-struct
    nx = Opts.nx;
    nu = Opts.nu;
    I = Opts.I;
    u_max = Opts.uMax;
    u_min = Opts.uMin;
    
    % extract parallelotop parameters
    cr = center(zonoR);
    Gr = generators(zonoR);
    
    % compute corner states of the parallelotope
    xCorner = symExtremePoints(zonoR,I);
    
    
    %% Linearize Exact Control Law
    
    % compute symbolic function and jacobian
    x = sym('x', [nx 1]);
    funcSymb = exactControlLaw(x,zonoR,uCorner(:,:,1));
    jacoSymb = jacobian(funcSymb,x);
    
    % evaluate function and jacobian at center trajectory  
    funcSymb = subs(funcSymb, x, cr);
    func = eval(funcSymb);
    
    jacoSymb = subs(jacoSymb, x, cr);
    jaco = eval(jacoSymb);
    
    hessSymb = cell(nu,1);
    hess = cell(nu,1);
    for i = 1:nu
        hessSymb{i} =  hessian(funcSymb(i),x);
        hessSymb{i} = subs(hessSymb{i}, x, cr);
        hess{i} = eval(hessSymb{i});
    end
    
    % convert linearized system to vector for the quadratic program
    xLin = cell(nu,1);
    for i = 1:nu
        xLin{i} = [jaco(i,:)';func(i)];
    end
    
    % compute matrizes for the quadratic program
    E1 = [2*diag(cr),eye(nx),zeros(nx,1)];
    E2 = [cr'.^2,cr',1];
    E = [E1;E2];
    
    Hex = {};
    fex = {};
    
    for i = 1:nu
       Hex{i} = 2 * (E' * E);
       fex{i} = - 2 * xLin{i}' * E; 
    end   
    
    % weight of the difference betwen linearized and input zonotope
    lambda = Opts.approx.lambda;
 
    
    %% Matrices for the quadratric Program (corner)

    Hcorn = cell(nu,1);
    fcorn = cell(nu,1);
    C = cell(nu,1);
    d = cell(nu,1);
    dim = 2*nx + 1;

    for i = 1:nu
        Htemp = zeros(dim,dim);
        ftemp = zeros(dim,1);
        Ctemp = zeros(2*2^nx,dim);
        dtemp = zeros(2*2^nx,1);

        counter = 1;

        for j = 1:2^nx
            x = xCorner(:,j);
            Etemp = x'.^2;
            Etemp = [Etemp,x',1];

            Htemp = Htemp + 2 * (Etemp' * Etemp);
            ftemp = ftemp - 2 * uCorner(i,j) * Etemp';

            Ctemp(counter,:) = Etemp;
            Ctemp(counter+1,:) = -Etemp;
            dtemp(counter) = u_max(i);
            dtemp(counter+1) = -u_min(i);

            counter = counter + 2;
        end
        
        % Get major bending direction of the exact control law
        bend = sign(jaco(i,:)*hess{i}*jaco(i,:)'); 
        if bend == 0
           bend = 1; 
        end
        
        Cext = zeros(nx,dim);
        dext = ones(nx,1) * 1e-4 * (-bend);

        for j = 1:length(dext)
            e = zeros(dim,1);
            e(j) = bend;
            Cext(j,:) = e';
        end

        fcorn{i} = ftemp;
        Hcorn{i} = Htemp;
        C{i} = [Ctemp;Cext];
        d{i} = [dtemp;dext];
    end


    %% Trade-off betwen corner and exact control law
    
    H = cell(nu,1);
    f = cell(nu,1);
    
    for i = 1:nu
        H{i} = (1-lambda) * Hcorn{i} + lambda * Hex{i};
        f{i} = (1-lambda) * fcorn{i} + lambda * fex{i}';
    end
    
    
    %% Solve quadratic program
    
    Aquad = cell(nu,1);
    bquad = cell(nu,1);
    oquad = cell(nu,1);
    
    options = optimoptions('quadprog','Display','off');

    for i = 1:nu
        y = quadprog(H{i},f{i}',C{i},d{i},[],[],[],[],[],options);

        Aquad{i} = diag(y(1:nx));
        bquad{i} = y(nx+1:end-1);
        oquad{i} = y(end);
    end
    
    
    %% Check if the solution is valid
    
    valid = ones(nu,1);
    
    for i = 1:nu
        xOpt = -0.5*(Aquad{i}\bquad{i});
        
        alpha = abs(Gr\(xOpt-cr));
        value = xOpt'*Aquad{i}*xOpt + bquad{i}'*xOpt + oquad{i};

        if value > u_max(i) || value < u_min(i)
           valid(i) = 0; 
        end
    end
    
    
    %% Compute linear control law
    
    [c,G] = computeInputZonotope(uCorner,zeros(nu,1),zonoR,Opts);
    

    %% Solve with fmincon
    
    Afmin = cell(nu,1);
    bfmin = cell(nu,1);
    ofmin = cell(nu,1);

    for i = 1:nu
        if valid(i)
        
            Afmin{i} = Aquad{i};
            bfmin{i} = bquad{i};
            ofmin{i} = oquad{i};
            
        else
            
            % Solve with fmincon
            obj = @(z) 0.5 * z' * H{i} * z + f{i}'*z;

            y0 = zeros(dim,1);
            y0(1:nx) = diag(Aquad{i});
            y0(nx+1:end-1) = bquad{i};
            y0(end) = oquad{i};

            options = optimoptions('fmincon','MaxFunEvals',50000,'Algorithm',...
                                 'sqp','Display','off','TolCon',1e-10); 

            [yNew,~,flag] = fmincon(obj, y0, C{i}, d{i},[],[],[],[],@(x)constraintfunctionQuad(x,nx,u_max(i),u_min(i)),options);
            
            if flag < 1
                % Optimization failed, use linear control law                
                temp = G/Gr;
                bfmin{i} = temp(i,:)';
                
                temp = c - temp*cr;
                ofmin{i} = temp(i);
                
                Afmin{i} = zeros(nx);
            else
                % Get matrices of the quadratic control law
                Afmin{i} = diag(yNew(1:nx));
                bfmin{i} = yNew(nx+1:end-1);
                ofmin{i} = yNew(end);
            end
        end
    end
    
    

    %% Package parameters of the quadratic control law to one vector
    
    params = zeros(nu*(2*nx+1),1);
    counter = 1;
    for i = 1:nu
       params(counter:counter+nx-1) = diag(Afmin{i});
       counter = counter + nx;
       params(counter:counter+nx-1) = bfmin{i};
       counter = counter + nx;
       params(counter) = ofmin{i};
       counter = counter + 1;
    end
end