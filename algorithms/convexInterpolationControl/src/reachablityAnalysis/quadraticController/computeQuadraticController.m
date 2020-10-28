function [zonoBig,reachSet,controlLawParam] = computeQuadraticController(dynamic,zono,zonoR,uCorner,Opts)
% COMPUTEQUADRATICCONTROLLER - implementation of the quadratic control law
%
% Syntax:
%       [zonoBig,reachSet,controlLawParam] = 
%               COMPUTEQUADRATICCONTROLLER(dynamic,zono,zonoR,uCorner,Opts)
%
% Description:
%       This function implements the quadratic control law. The
%       reachable set is computed for one time step of the center
%       trajectory, which consists of multiple steps of the corner
%       trajectories. 
%
% Input Arguments:
%
%       -dynamic:       object containing the system dynamics (class:
%                       nonlinParamSys)
%       -zono:          initial zonotope (class zonotope or polyZonotope)
%       -zonoR:         parallelotope which is an overapproximation of the
%                       initial zonotope zono
%       -uCorner:       optimal control inputs for the corner trajectories
%                       (dimension: [nu,nx^2,Opts.Ninter])
%       -Opts:          structure containing user defined options for the 
%                       algorithm           
%
% Output Arguments:
%
%       -zonoBig:           extended zonotope that results from the 
%                           reachability analysis. This zonotope still
%                           includes the auxiliary states
%       -reachSet:          cell array containing the reachable sets of all
%                           intermediate time steps
%       -controlLawParam:   parameters of the computed control law
%
% See Also:
%       convexInterpolationControl, computeExactController
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
    
    % parallelotope              
    Gr = generators(zonoR);
    cr = center(zonoR);
    
    % extract zonotope parameters
    if isa(zono,'zonotope')                 % linear zonotope                  
        G = generators(zono);
        c = center(zono);
    else                                    % polynomial zonotope
        c = zono.c;
        G = zono.G;
        Grest = zono.Grest;
    end
    
    % extract model dimensions
    nx = Opts.nx;
    nu = Opts.nu;

    
     %% Compute data needed for control law implemtation
    
    % The linear part of the control law can be added to the initial
    % zonotope. To do this, the initial zonotope is extended with auxiliary
    % states, which represent the transformation according to the linear
    % formula    
    
     if isa(zono,'zonotope')                             % linear zonotope 
        % compute linear part of the control law
        c_ = cr;
        G_ = Gr * (Gr\G);

        % construct the extended linear zonotope for this time step
        cExtended = [c;c_];
        GExtended = [G;G_];
        R0 = zonotope([cExtended,GExtended]);
        
    else                                                % polynomial zonotope
        % compute linear part of the control law
        if isempty(Grest)
           Grest = zeros(nx,1); 
        end
        
        c_ = cr;
        G_ = Gr * (Gr\G);
        Grest_ = Gr * (Gr\Grest);
        
        % construct the extendet polynomial zonotope for this time step
        cExt = [c;c_];
        GExt = [G;G_];
        GrestExt = [Grest;Grest_];

        R0 = polyZonotope(cExt,GExt,GrestExt,zono.expMat); 
    end
    
    
    %% Compute Control Law
    
    params = zeros(nu*(2*nx+1),Opts.Ninter);
    
    for i = 1:Opts.Ninter
        [params(:,i)] = computeQuadraticControlLaw(uCorner(:,:,i),zonoR,Opts);
    end
   
    
    
    %% Reachable set computation
    
    % compute reachable set for one time step of the center trajectory
    [zonoBig,reachSet] = reachExactController(dynamic,R0,params,Opts);

    
    
    %% Postprocessing
    
    % store data needed for control law implementation
    A = cell(Opts.Ninter,1);
    b = cell(Opts.Ninter,1);
    o = cell(Opts.Ninter,1);
 
    for j = 1:Opts.Ninter
        
        A{j} = cell(nu,1);
        b{j} = cell(nu,1);
        o{j} = cell(nu,1);
        index = 1;
        
        for i = 1:nu
            A{j}{i} = diag(params(index:index+nx-1,j));
            index = index + nx;
            b{j}{i} = params(index:index+nx-1,j);
            index = index + nx;
            o{j}{i} = params(index,j);
            index = index + 1;
        end
    end
    
    controlLawParam.A = A;
    controlLawParam.b = b;
    controlLawParam.o = o;
end

