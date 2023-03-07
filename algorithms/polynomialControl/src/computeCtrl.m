function pval = computeCtrl(p0,Rparam,alpha_c,Opts)
% COMPUTECTRL - determine optimal control parameters via optimization
%
% Syntax:
%       pval = COMPUTECTRL(p0,Rparam,alpha_c,Opts)
%
% Description:
%       Computes the optimal control parameters by solving the optimization
%       problem (7) in [1].
%
% Input Arguments:
%
%       -p0:        initial guess for the control parameter
%       -Rparam:    parameterized reachable set
%       -alpha_c:   zonotope factors for the reference trajectory inputs
%       -Opts:      structure containing all options
%
% Output Arguments:
%
%       -pval:  optimal value for the controller parameter
%
% See Also:
%       polynomialControl
%
% References:
%       * *[1] Gassman et al. (2021)*, Verified Polynomial Controller 
%              Synthesis for Disturbed Nonlinear Systems
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % form objective poly. zonotope by substracting reference trajectory 
    len = Opts.extHorizon.length;
    
    for i = 1:len
        Rparam{i} = -Opts.Xf(:,1+i) + Rparam{i};
    end
    
    % compute constraints for the parameter values
    [A_p,b_p] = constraintsControlParam(Opts.P);
    
    % use reference trajectory for constant control input
    if Opts.refInput

        ii0 = 1:Opts.np_s:Opts.Np*len;
        p0(ii0) = [];
        iMask = ismember(1:Opts.Np*len,ii0);
        b_p = b_p - A_p(:,ii0)*alpha_c(:);

        A_p(:,ii0) = [];
        Opts.Idp(1:Opts.np_s:end,:) = [];

        % Rparam already has the appropriate ids missing from
        % reachPolyParam.m
    end
    
    Idp = Opts.Idp(:,1:len);
    idp = Idp(:);
    
    % over-approximate (partly) as zonotope
    for i = 1:length(Rparam)
        % if we ever want constraints, make sure that Grest makes it into
        % pZ_cell
        Rparam{i} = reduce(Rparam{i},'pca',30);
    end

    pZ_cell = extractPolyGenerators(Rparam,Opts);
    pZ_t = restoreId(stack(pZ_cell{:}),idp);
    
    % auxiliary variables
    Np = length(idp);
    Nt = length(center(pZ_t));
    Nz = size(A_p,2) - Np;

    TOL_OPT = 1e-8;

    % inequality constraints
    A = [zeros(size(A_p,1),Nt),A_p];
    b = b_p;
   
    % bounds
    lb = [zeros(Nt,1);-ones(Np,1);zeros(Nz,1)]-1e-6;
    ub = [inf(Nt,1);ones(Np,1);ones(Nz,1)]+1e-6;

    % (non-)linear constraints (1-norm resolved)
    f_t = fhandle(pZ_t,{idp}); 

    f_ineq = @(x) [f_t(x(Nt+(1:Np)));-f_t(x(Nt+(1:Np)))]-[x(1:Nt);x(1:Nt)];

    % jacobian (fmincon needs transpose of jacobian)
    g_t = jacobianHandle(pZ_t,idp,setdiff(idp,pZ_t.id));

    g_ineq = @(x) [-eye(Nt),-eye(Nt); ...
                   g_t(x(Nt+(1:Np)))',-g_t(x(Nt+(1:Np)))'; ...
                   zeros(Nz,2*Nt)];

    % initial value
    t0 = abs(f_t(p0));
    [p0,z0] = strictFeasZ(p0,A_p,b_p,0);
    x0 = [t0;p0;z0];
    
    % objective function (objective + gradient)
    fobj = @(x) sum(x(1:Nt));
    gobj = @(x) [ones(Nt,1);zeros(Np+Nz,1)];
    
    f = @(x) objfunc(x,fobj,gobj);
    
    % nonlinear constraints (no equality constraints)
    nonlconstr = @(x) constrfunc(x,f_ineq,g_ineq);
    
    % settings for optimization algorithm
    options = optimoptions(@fmincon,'Algorithm','sqp',...
              'SpecifyObjectiveGradient',true, ...
              'SpecifyConstraintGradient',true, 'Display','none',...
              'OptimalityTolerance',TOL_OPT, ...
              'MaxFunctionEvaluations',1000, 'maxIterations',1000);
    
    % solve optimization problem
    [x,~,exitflag] = fmincon(f,x0,A,b,[],[],lb,ub,nonlconstr,options);
    
    if exitflag < 0
        disp('Error during optimization of control parameters - using initial point');
        pval = p0;
    else
        pval = x(Nt+(1:Np));
    end
    
    % exchange values for constant parameters
    if Opts.refInput
        pval_ = zeros(length(iMask),1);
        pval_(iMask) = alpha_c(:);
        pval_(~iMask) = pval;
        pval = pval_;
    end
end


% Auxiliary Functions -----------------------------------------------------

function [fval,gval] = objfunc(x,fobj,gobj)
    fval = fobj(x);
    if nargout>1
        gval = gobj(x);
    end
end

function [c_ineq,c_eq,g_ineq,g_eq] = constrfunc(x,f_ineq,fg_ineq)
    c_ineq = f_ineq(x);
    c_eq = [];
    g_ineq = fg_ineq(x);
    g_eq = [];
end