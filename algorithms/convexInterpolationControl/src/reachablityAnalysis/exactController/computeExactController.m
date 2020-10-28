function [zonoBig,reachSet,controlLawParam] = computeExactController(dynamic,zono,zonoR,uCorner,Opts)
% COMPUTEEXACTCONTROLLER - implementation of the exact control law
%
% Syntax:
%       [zonoBig,reachSet,controlLawParam] = 
%                   computeExactController(dynamic,zono,zonoR,uCorner,Opts)
%
% Description:
%       This function implements the control law that uses the exact
%       solution for the convex combinations of the corner inputs. The
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
%                       initial zonotope zono (class zonotope)
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


    %% Extract parameter

    % parallelotope
    Zr = zonoR.Z;                 
    Gr = Zr(:,2:end);
    cr = Zr(:,1);
    
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
    nx = length(c);
    nu = size(uCorner,1);
    nParam = size(uCorner,2);
    
    
    %% Compute data needed for control law implemtation
    
    % The linear part of the control law can be added to the initial
    % zonotope. To do this, the initial zonotope is extended with auxiliary
    % states, which represent the transformation according to the linear
    % formula

    if isa(zono,'zonotope')                             % linear zonotope 
        % compute linear part of the control law
        c_ = 0.5 * (Gr\(c-cr)) + 0.5 * ones(nx,1);
        G_ = 0.5 * (Gr\G);
        
        % construct the extended linear zonotope for this time step
        cExtended = [c;c_];
        GExtended = [G;G_];
        R0 = zonotope([cExtended,GExtended]);
        
    else                                                % polynomial zonotope
        % compute linear part of the control law
        if isempty(Grest)
           Grest = zeros(nx,1); 
        end
        
        c_ = 0.5 * (Gr\(c-cr)) + 0.5 * ones(nx,1);
        G_ = 0.5 * (Gr\G);
        Grest_ = 0.5 * (Gr\Grest);
        
        % construct the extended polynomial zonotope for this time step
        cExt = [c;c_];
        GExt = [G;G_];
        GrestExt = [Grest;Grest_];

        R0 = polyZonotope(cExt,GExt,GrestExt,zono.expMat);      
    end
    
    
    %% Store data needed for control law implementation
    
    % store the optimal input values for all corner points in the parameter
    % vector
    
    param = zeros(nParam,Opts.Ninter);
    
    for j = 1:Opts.Ninter
        index = 1;
        for i = 1:size(uCorner,2)                        
           param(index:index+nu-1,j) = uCorner(:,i,j);
           index = index + nu;
        end
    end
    
    
    
    %% Reachable set computation

    % compute reachable set for one time step of the center trajectory 
    [zonoBig,reachSet] = reachExactController(dynamic,R0,param,Opts);
    
    
    
    %% Postprocessing
    
    % store data needed for control law implementation
    controlLawParam = cell(Opts.Ninter,1);
    for i = 1:Opts.Ninter
       controlLawParam{i} = squeeze(uCorner(:,:,i));
    end
end