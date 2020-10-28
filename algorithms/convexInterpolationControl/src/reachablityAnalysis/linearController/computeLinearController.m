function [zonoBig,reachSet,controlLawParam] = computeLinearController(dynamic,zono,zonoR,uCorner,uCenter,Opts)
% COMPUTELINEARCONTROLLER - implementation of the linear control law
%
% Syntax:
%       [zonoBig,reachSet,controlLawParam] = ...
%          COMPUTELINEARCONTROLLER(dynamic,zono,zonoR,uCorner,uCenter,Opts)
%
% Description:
%       This function implements the control law that uses a linear 
%       approximation of the convex combinations of the corner inputs. The
%       reachable set is computed for one time step of the center
%       trajectory, which consists of multiple steps of the corner
%       trajectories. The function implements Alg. 2 in [1]. 
%
% Input Arguments:
%
%       -dynamic:       object containing the system dynamics (class:
%                       nonlinearSys)
%       -zono:          initial zonotope (class zonotope or polyZonotope)
%       -zonoR:         parallelotope which is an overapproximation of the
%                       initial zonotope zono (class zonotope)
%       -uCorner:       optimal control inputs for the corner trajectories
%                       (dimension: [nu,nx^2,Opts.Ninter])
%       -uCenter:       optimal control inputs for the center trajectory
%                       (dimension: [nx, Opts.Nc])
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


    %% Compute Input Zonotopes
    
    % initialize variables
    cu = cell(Opts.Ninter,1);
    Gu = cell(Opts.Ninter,1);
    
    % compute input zonotopes for each time step 
    %(see Line 2 of Alg. 2 in [1])
    for i = 1:Opts.Ninter
        [cu{i},Gu{i}] = computeInputZonotope(uCorner(:,:,i),uCenter,zonoR,Opts);
    end

    
    %% Compute Input Zonotope to Parallelotope assignment

    % compute the a matrix encoding the information which input
    % generators correspond to which state generators, when we switch
    % the input values during reachability analysis. Is used to make
    % reachability analysis easier (see Eq. (15) in [1])
    
    % parallelotope parameters        
    Gr = generators(zonoR);
    
    if isa(zono,'zonotope')                             % linear zonotope
        % extract zonotope parameters         
        G = generators(zono);

        % compute input-zonotope to state zonotope assignment
        inputAssign=Gr\(G);
        
    else      
        % compute input-zonotpe to state zonotope assignment
        inputAssign.G = Gr\(zono.G);
        if isempty(zono.Grest)
            inputAssign.Grest = Gr\zeros(Opts.nx,1);
            zono = polyZonotope(zono.c,zono.G,zeros(Opts.nx,1),zono.expMat);
        else
            inputAssign.Grest = Gr\(zono.Grest);
        end
    end
    
    
    %% Reachability Analysis
    
    % compute the reachable set of the controlled system
    % (see Line 3 of Alg. 2 in [1])
    [zonoBig,reachSet] = reachLinearController(dynamic,zono,cu,Gu,...
                                               inputAssign,Opts);
    
    
    %% Postprocessing
    
    % Compute linear control law u = Ax+b for offline phase
    A = cell(Opts.Ninter,1);
    b = cell(Opts.Ninter,1);
    
    for i = 1:Opts.Ninter
        A{i} = Gu{i}*inv(Gr);
        b{i} = cu{i} - A{i} * center(zonoR);
    end
    
    controlLawParam.A = A;
    controlLawParam.b = b;
