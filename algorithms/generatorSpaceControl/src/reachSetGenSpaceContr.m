function [reachSet,R] = reachSetGenSpaceContr(sys,R,P,alpha,Opts)
% REACHSETGENSPACECONTR - computes the reachable set for one time step
%
% Syntax:
%       [reachSet,R] = REACHSETGENSPACECONTR(sys,R,P,alpha,Opts)
%
% Description:
%       This function computes the reachable set of the controlled system
%       for one time step of the controller that is based on optimal 
%       control in generator space.     
%
% Input Arguments:
%
%       -sys:       object with the closed-loop dynamics 
%                   (class: nonlinearSys)
%       -R:         initial set (class: zonotope)
%       -P:         parallelotope enclosure of the initial set 
%                   (class: zonotope)
%       -alpha:     cell-array storing the control law for the time step
%       -Opts:      a struct containing the following fields
%
%           -.Ninter:       number of intermediate time steps
%           -.reachSteps:   number of reachability steps in each time step
%           -.U:            set of admissible control inputs
%           -.ReachOpts:    settings for reachability analysis with CORA
%           -.nx:           number of states
%           -.nu:           number of inputs
%
% Output Arguments:
%
%       - reachSet: object of class reachSet storing the reachable set
%       - R:        final reachable set 
%                   
% See Also:
%       generatorSpaceControl
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
% Authors:      Jan Wagener, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization
    options = Opts.ReachOpts;
    params = Opts.ReachParams;
    reachSet = []; 
    tStart = Opts.tStart;

    % get input zonotope parameters
    cu = center(Opts.U);
    Gu = generators(Opts.U);

    % compute input-zonotope to state zonotope assignment
    Gp = generators(P);
    c = center(R);
    
    if isa(R,'zonotope')
        G = generators(R);
    else
        G = [R.G,R.Grest]; 
        E = R.expMat;
    end
    
    inputAssign = Gp\G;
    
    % compute measurement error set
    if ~isempty(Opts.V)
        Opts.V = zonotope(Gp\Opts.V.Z);
    end
    
    % loop over all intermediate time steps
    for i = 1:Opts.Ninter
        
        % get current control law
        temp = alpha{i};
        alpha_u = temp(:,1);
        alpha_g = temp(:,2:end);
        
        % compute extended initial set
        if isempty(Opts.V)
            c_ = [c; cu+Gu*alpha_u; zeros(size(inputAssign,1),1)];
            G_ = [G; Gu*alpha_g*inputAssign; inputAssign]; 
        else
            c_ = [c; zeros(Opts.nx,1)];
            G_ = [G; inputAssign];
        end

        if isa(R,'zonotope')
            R0 = zonotope([c_,G_]); 
        else
            R0 = polyZonotope(c_,G_(:,1:size(E,2)),G_(:,size(E,2)+1:end),E); 
        end
        
        % add set of measurement errors
        if ~isempty(Opts.V)
           R0 = cartProd(R0,Opts.V);
           p = [cu+Gu*alpha_u; reshape(Gu*alpha_g,[Opts.nu*Opts.nx,1])];
           params.paramInt = p;
        end
        
        % update time and initial set
        params.R0 = R0;
        params.tStart = tStart;
        params.tFinal = params.tStart + Opts.dt;
        
        % compute reachable set
        Rtemp = reach(sys,params,options); 
        
        reachSet = add(reachSet,Rtemp);
        tStart = params.tFinal;
        
        % get updated parameters for the initial set
        Rfin = Rtemp.timePoint.set{end};
        
        if isa(R,'zonotope')
            G_ = generators(Rfin);
        else
            G_ = [Rfin.G,Rfin.Grest];
            E = Rfin.expMat;
        end
        
        c_ = center(Rfin);
        c = c_(1:Opts.nx);
        G = G_(1:Opts.nx,:);
        
        if isempty(Opts.V)
            inputAssign = G_(Opts.nx+Opts.nu+1:Opts.nx+Opts.nu+Opts.nx,:); 
        else
            inputAssign = G_(Opts.nx+1:2*Opts.nx,:); 
        end
    end
    
    % construct final reachable set
    if isa(R,'zonotope')
        R = zonotope([c,G]);
    else
        R = polyZonotope(c,G(:,1:size(E,2)),G(:,size(E,2)+1:end),E);  
    end