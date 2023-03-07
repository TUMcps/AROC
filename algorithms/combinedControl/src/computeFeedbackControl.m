function [K,R] = computeFeedbackControl(sys,objCtrl_ff,Opts)
% COMPUTEFEEDBACKCONTROL - compute feedback matrix using optimization
%
% Syntax:
%       [K,R] = COMPUTEFEEDBACKCONTROL(sys,objCtrl_ff,Opts)
%
% Description:
%       Computes an optimal feedback matrix for the combined control
%       algorithm by optimization over the reachable set of the controlled
%       system.
%
% Input Arguments:
%
%       -sys:           controlled system object (class: nonlinParamSys)
%       -objCtrl_ff:    feedforward contr. object (class: objGenSpaceContr 
%                       or objPolyContr)
%       -Opts:          a structure containing the algorithm settings
%
% Output Arguments:
%
%       -K:             cell-array storing the resulting feedback matrices
%       -R:             resulting reachable set (class: reachSet) 
%
% See Also:
%       combinedControl
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Syntesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Victor Gassmann, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Arificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialize reachable set computation to precompute tensors for CORA
    % -> speeds-up optimiaztion over reachable sets
    if isa(objCtrl_ff,'objGenSpaceContr')
        initReach(sys,objCtrl_ff,Opts);
    else
        initReachPoly(sys,objCtrl_ff,Opts);
    end
    
    % settings for nonlinear optimization with fmincon
    fopt = @(x,m) objfun(x,sys,objCtrl_ff,Opts,m);
    fcon = @(x,m) constr(x,sys,objCtrl_ff,Opts,m);
    
    Cstr_TOL = 1e-6;
    fopts = optimoptions('fmincon','MaxFunEvals',2000,'MaxIter',...
                        Opts.maxIter,'Display','off','Algorithm','sqp',...
                        'ConstraintTolerance',Cstr_TOL,'UseParallel',true);
                    
    x0 = [ones(Opts.nx-1,1);ones(Opts.nu,1)];
    
    lb = 1/Opts.bound*ones(length(x0),1);
    ub = Opts.bound*ones(length(x0),1);
    
    % To speed-up the optimization we use a warm-start inititialization,
    % where obtain the warm-start initial solution from simpler approximate
    % optimization problems

    % Step 1: optimization using linearized system
    if true
        mode = 'lin';
        fopts_lin = fopts;
        fopts_lin.MaxIter = 10;

        x0 = fmincon(@(x) fopt(x,mode),x0,[],[],[],[], ...
                     lb,ub,@(x) fcon(x,mode),fopts_lin);
    end

    % Step 2: optimization using nonlinear system without abstraction error
    if false
        mode = 'nonlin_noErr';
        fopts_nl_nE = fopts;
        fopts_nl_nE.MaxIter = 3;

        x0 = fmincon(@(x) fopt(x,mode),x0,[],[],[],[], ...
                     lb,ub,@(x) fcon(x,mode),fopts_nl_nE);
    end

    % Step 3: optimization using nonlinear system with abstraction error 
    %         from previous time step
    if true
        mode = 'nonlin_prevErr';
        fopts_nl_pE = fopts;
        fopts_nl_pE.MaxIter = 3;

        x0 = fmincon(@(x) fopt(x,mode),x0,[],[],[],[], ...
                     lb,ub,@(x) fcon(x,mode),fopts_nl_pE);
    end

    % Step 4: optimization using nonlinear system with "correct" lin. error
    mode = 'nonlin';
    [x_sol,~,exitflag] = fmincon(@(x) fopt(x,mode),x0,[],[],[],[], ...
                                 lb,ub,@(x) fcon(x,mode),fopts);
    if exitflag < 0
        error('Something wrong with optimization!');
    end

    % compute the final reachable set with smaller time step size
    Opts.ReachOpts.timeStep = Opts.dt/Opts.reachStepsFin;
    
    if isa(objCtrl_ff,'objGenSpaceContr')
        [R,U,K] = CompReach(x_sol,sys,objCtrl_ff,Opts,mode);
    else
        [R,U,K] = CompReachPoly(x_sol,sys,objCtrl_ff,Opts,mode);
    end
    
    [c,~] = Constraints(R,U,Opts);
    if any(c > 0) 
        error('Failed to find a feasible solution!'); 
    end
end


% Auxiliary Functions -----------------------------------------------------

function f = objfun(x,sys,objCtrl_ff,Opts,mode)
% objective function for nonlinear optimization

    try
        % compute reachable set
        if isa(objCtrl_ff,'objGenSpaceContr')
            [R,U] = CompReach(x,sys,objCtrl_ff,Opts,mode);
        else
            [R,U] = CompReachPoly(x,sys,objCtrl_ff,Opts,mode);
        end

        Rtp = R(end).timePoint.set{end};
        
        % compute value for the objective function
        f = Objective(Rtp,U,Opts.Q,Opts.R);
    catch
        f = inf;
    end
end

function [c_ineq,c_eq] = constr(x,sys,objCtrl_ff,Opts,mode)
% constraint function for nonlinear optimization

    try
        % compute reachable set
        if isa(objCtrl_ff,'objGenSpaceContr')
            [R,U] = CompReach(x,sys,objCtrl_ff,Opts,mode);
        else
            [R,U] = CompReachPoly(x,sys,objCtrl_ff,Opts,mode);
        end
            
        % check if the constraints are satisfied
        [c_ineq,c_eq] = Constraints(R,U,Opts);
    catch
       c_ineq = inf(Opts.nIneq,1);
       c_eq = [];
    end
end

function [RS,U,K] = CompReach(x,sys,objCtrl_ff,Opts,mode)
% compute the reachable set of the controlled system

    % get parameter
    nu = Opts.nu; nx = Opts.nx; nw = Opts.nw;
    params = Opts.ReachParams;

    % construct weighting matrices for LQR controller 
    temp = [1;x];
    Q = diag(temp(1:nx));
    R = diag(temp(nx+1:nx+nu));

    % initialization
    RS = [];
    K = cell(Opts.N,1);
    U = cell(Opts.N,Opts.reachSteps);
    
    % compute intial factor-to-generator assignment matrix
    Gbeta = generators(objCtrl_ff.parallelo{1})\ ...
            generators(zonotope(objCtrl_ff.R0));

    % construct extended initial set
    Ztp_beta = zonotope([zeros(nx,1),Gbeta]);
    Ztp_x = zonotope(Opts.R0);
    Zx = Ztp_x.Z;
    Ztp_xffc = zonotope([Zx(:,1),zeros(nx,size(Zx,2)-1)]);
    Ztp_dxff = zonotope([zeros(nx,1),Zx(:,2:end)]);
    Gu_ff = generators(zonotope(objCtrl_ff.U));
    
    Ztp = zonotope([Ztp_x.Z;Ztp_xffc.Z;Ztp_dxff.Z;Ztp_beta.Z]);
    
    if ~strcmp(mode,'lin') && strcmp(Opts.ReachOpts.alg,'poly')
       Ztp = polyZonotope(Ztp); 
    end

    % loop over all time steps
    for i = 1:Opts.N

        % compute control law
        Alpha = objCtrl_ff.alpha{1}{i};
        GuffAlpha_i = Gu_ff*Alpha(:,2:end);
        uff_c = Gu_ff*Alpha(:,1);

        % compute linearized system matrices
        xlin = 1/2*(Opts.refTraj.xc(:,i)+Opts.refTraj.xc(:,i+1));
        ulin = Opts.refTraj.uc(:,i);
        w = zeros(Opts.nw,1);
        Ai = Opts.linDyn.A(xlin,ulin,w);
        Bi = Opts.linDyn.B(xlin,ulin,w);
        Bwi = Opts.linDyn.Bw(xlin,ulin,w);
        assert(rank(ctrb(Ai,Bi))==nx,'Linearized system not controllable!');

        % compute feedback matrix K with LQR controller
        Ki = -lqr(Ai,Bi,Q,R);
        K{i} = Ki;
        
        % reachability setup
        params.R0 = Ztp;
        params.tStart = (i-1)*Opts.dt;
        params.tFinal = i*Opts.dt;

        % different reachability algorithms depending on the current mode
        if strcmp(mode,'lin')
            
            % extendet system state x = [x_orig, xffc, dxff, beta]
            A_ext = [Ai+Bi*Ki, -Bi*Ki, -Bi*Ki, Bi*GuffAlpha_i; 
                     zeros(nx), Ai, zeros(nx), zeros(nx);  
                     zeros(nx),  zeros(nx), Ai, Bi*GuffAlpha_i; 
                     zeros(nx,4*nx)];
               
            % disturbance only for x
            B_ext = [Bwi;zeros(3*nx,nw)];
            
            % constant offset
            c_ext = [Bi*uff_c;Bi*uff_c;zeros(2*nx,1)];
            
            % construct linear system
            sys = linearSys(A_ext,B_ext,c_ext);
            
            % reachability options
            options = Opts.ReachOptsLin;
            
            % compute reachable set
            R_ext = reach(sys,params,options);
            RS = add(RS,projectReachSet(R_ext,1:nx));
            
        else
            % parameter values and reachability options
            params.paramInt = [Ki(:);xlin;ulin;uff_c;GuffAlpha_i(:)];
            options = Opts.ReachOpts;
            
            % consider measurement errors
            if ~isempty(Opts.V)
               params.U = cartProd(Opts.W,Opts.V); 
               params.R0 = cartProd(params.R0,Opts.V);
            end
            
            % method to compute abstraction error
            if strcmp(mode,'nonlin')
                options.approxErr = false;
            elseif strcmp(mode,'nonlin_prevErr')
                options.approxErr = true;
                options.prevErrScale = 0.85;
            elseif strcmp(mode,'nonlin_noErr')
                options.prevErr = zonotope(zeros(4*nx,1));
            end
            
            % compute reachable set
            R_ext = reachNonlinear(sys,params,options);
            RS = add(RS,projectReachSet(R_ext,1:nx));
        end

        % update initial set
        Rtp_ext = R_ext.timePoint.set{end};
        Ztp = project(Rtp_ext,1:4*nx);

        % save set of applied control inputs U
        for j = 1:length(R_ext.timeInterval.set)
            Rti_j = zonotope(R_ext.timeInterval.set{j});
            Z_x = Rti_j.Z(1:nx,:);
            Z_xff = Rti_j.Z(nx+1:2*nx,:)+Rti_j.Z(2*nx+1:3*nx,:);
            Z_beta = Rti_j.Z(3*nx+1:4*nx,:);
            Z_uff = GuffAlpha_i*Z_beta;
            Z_uff(:,1) = Z_uff(:,1) + ulin; 
            U{i,j} = zonotope(Z_uff+Ki*(Z_x-Z_xff));
            if ~isempty(Opts.V)
               U{i,j} = U{i,j} + Ki*Opts.V; 
            end
        end
    end
end

function [RS,U,K] = CompReachPoly(x,sys,objCtrl_ff,Opts,mode)
% compute the reachable set of the controlled system

    % get parameter
    nu = Opts.nu; nx = Opts.nx; nw = Opts.nw;
    params = Opts.ReachParams;
    rs = round(Opts.tFinal/Opts.ReachOpts.timeStep/Opts.N);
    
    % construct weighting matrices for LQR controller 
    temp = [1;x];
    Q = diag(temp(1:nx));
    R = diag(temp(nx+1:nx+nu));

    % initialization
    RS = [];
    K = cell(Opts.N,1);
    U = cell(Opts.N,rs);
    
    % compute intial factor-to-generator assignment matrix
    Gbeta = generators(objCtrl_ff.parallelo{1})\ ...
                generators(zonotope(objCtrl_ff.R0));

    % construct extended initial set
    Uff_x = objCtrl_ff.Ctrl_x{1}; idv = Uff_x.id; o = zeros(nx,0);

    R0 = polyZonotope(Opts.R0);

    Uff = subs(Uff_x,R0,idv);
    R_xff = polyZonotope(center(R0),o,o,o,idv);
    R_dxff = R0 + (-center(R0));

    if ~isempty(Opts.V)
        tmp = subs(-center(Uff_x) + Uff_x,polyZonotope(Opts.V),idv);
        Uff = Uff + zonotope(tmp);
    end

    Rtp_ext = stack(R0,R_xff,R_dxff);
    
    if strcmp(mode,'lin')
        Rtp_no_beta = zonotope(project(Rtp_ext,1:3*nx));
        Rtp_ext = zonotope([Rtp_no_beta.Z;zeros(nx,1),Gbeta]);
    end

    % loop over all time steps
    for i = 1:Opts.N
        
        % compute linearized system matrices
        xlin = 1/2*(Opts.refTraj.xc(:,i)+Opts.refTraj.xc(:,i+1));
        ulin = Opts.refTraj.uc(:,i);
        Ai = Opts.linDyn.A(xlin,ulin,zeros(nw,1));
        Bi = Opts.linDyn.B(xlin,ulin,zeros(nw,1));
        Bwi = Opts.linDyn.Bw(xlin,ulin,zeros(nw,1));
        assert(rank(ctrb(Ai,Bi))==nx,'Linearized system not controllable!');

        % compute feedback matrix K with LQR controller
        Ki = -lqr(Ai,Bi,Q,R);
        K{i} = Ki;
                    
        % extract GenSpace parameters from linear polyController
        Uff_i = project(Uff,(i-1)*nu+1:i*nu);
        
        if strcmp(mode,'lin')
            [uff_c,GuffAlpha_i] = linPolyCtrl2GenSpace(Uff_i);
        else
            Rtp_ext = stack(project(Rtp_ext,1:3*nx),Uff_i);
            params.paramInt = [Ki(:);xlin;ulin];
        end
        
        % reachability setup
        params.R0 = Rtp_ext;
        params.tStart = (i-1)*Opts.dt;
        params.tFinal = i*Opts.dt;

        % different reachability algorithms depending on the current mode
        if strcmp(mode,'lin')
            
            % extendet system state x = [x_orig, xffc, dxff, beta]
            A_ext = [Ai+Bi*Ki, -Bi*Ki, -Bi*Ki, Bi*GuffAlpha_i; 
                     zeros(nx), Ai, zeros(nx), zeros(nx);  
                     zeros(nx),  zeros(nx), Ai, Bi*GuffAlpha_i; 
                     zeros(nx,4*nx)];
               
            % disturbance only for x
            B_ext = [Bwi;zeros(3*nx,nw)];
            
            % constant offset
            c_ext = [Bi*uff_c;Bi*uff_c;zeros(2*nx,1)];
            
            % construct linear system
            sys = linearSys(A_ext,B_ext,c_ext);
           
            params.R0 = Rtp_ext;
            
            % reachability options
            options = Opts.ReachOptsLin;
            
            % compute reachable set
            R_ext = reach(sys,params,options);
            RS = add(RS,projectReachSet(R_ext,1:nx));
            
        else
            options = Opts.ReachOpts;
            
            % consider measurement errors
            if isempty(Opts.V)
                params.U = zonotope(cartProd(Opts.W,zeros(nx,1)));
            else
                params.U = zonotope(cartProd(Opts.W,Opts.V)); 
            end
            
            % method to compute abstraction error
            if strcmp(mode,'nonlin')
                options.approxErr = false;
            elseif strcmp(mode,'nonlin_prevErr')
                options.approxErr = true;
                options.prevErrScale = 0.85;
            elseif strcmp(mode,'nonlin_noErr')
                if isempty(Opts.V)
                    options.prevErr = zonotope(zeros(4*nx,1));
                else
                    nv = length(center(Opts.V));
                    options.prevErr = zonotope(zeros(4*nx+nv,1));
                end
            end
            
            % compute reachable set
            R_ext = reachNonlinear(sys,params,options);
            RS = add(RS,projectReachSet(R_ext,1:nx));
        end

        % update initial set
        Rtp_ext = R_ext.timePoint.set{end};

        % save set of applied control inputs U
        for j = 1:length(R_ext.timeInterval.set)
            
            if strcmp(mode,'lin') 
                Rti_j = zonotope(R_ext.timeInterval.set{j});
                Z_x = Rti_j.Z(1:nx,:);
                Z_xff = Rti_j.Z(nx+1:2*nx,:)+Rti_j.Z(2*nx+1:3*nx,:);
                Z_beta = Rti_j.Z(3*nx+1:4*nx,:);
                Z_uff = [uff_c,GuffAlpha_i*Z_beta(:,2:end)]; 
                U{i,j} = zonotope(Z_uff+Ki*(Z_x-Z_xff));
                if ~isempty(Opts.V)
                   U{i,j} = U{i,j} + Ki*Opts.V; 
                end
            else
                Rti_j = R_ext.timeInterval.set{j};
                Rti_x = project(Rti_j,1:nx);
                Rti_xref = project(Rti_j,nx+1:2*nx);
                Rti_dxff = project(Rti_j,2*nx+1:3*nx);
                Rti_xff = exactPlus(Rti_xref,Rti_dxff);
                R_uctrl = exactPlus(Uff_i,Ki*exactPlus(Rti_x,-1*Rti_xff));
                U{i,j} = zonotope(R_uctrl);
            end
        end
    end
end

function fobj = Objective(Rtp,U,Q,R)
% compute value for the objective function from the final reachable set

    % cost of final states
    Int_x = interval(Rtp);
    cost_x = sum(Q*2*rad(Int_x));
    
    % input costs
    cost_u = 0;
    for i = 1:length(U)
        Int_u = interval(U{i});
        cost_u = cost_u + sum(R*2*rad(Int_u));
    end
    
    % overall costs
    fobj = cost_x + cost_u; 
end

function [c,c_eq] = Constraints(R,U,Opts)
% compute the constrained violation from the reachable set

    c = []; c_eq = [];

    % consider state constraints
    if ~isempty(Opts.stateCon)
        for i = 1:length(R.timeInterval.set)
            Rtemp = R.timeInterval.set{i};
            if ~isa(Rtemp,'zonotope')
               Rtemp = zonotope(Rtemp); 
            end
            c = ZonoConstraints(Opts.stateCon.A,Opts.stateCon.b,Rtemp);
        end
    end
    
    % consider constraint for final reachable set
    if ~isempty(Opts.finCon)
        Rfin = R(end).timePoint.set{end};
        if ~isa(Rfin,'zonotope')
           Rfin = zonotope(Rfin); 
        end
        c = [c; ZonoConstraints(Opts.finCon.A,Opts.finCon.b,Rfin)];
    end

    % consider input constraints
    for i = 1:numel(U)
        c = [c; ZonoConstraints(Opts.inputCon.A,Opts.inputCon.b,U{i})];
    end
end

function c = ZonoConstraints(C,d,Z)
% compute constraint violation for zonotope in polytope containment
    c = C*center(Z)+sum(C*abs(generators(Z)),2) - d;
end

function initReach(sys,objCtrl_ff,Opts)
% call CORA reach function once in advance so that the required tensors are
% calculated

    % construct initial set
    c = center(Opts.R0); G = generators(Opts.R0);
    
    Gbeta = generators(objCtrl_ff.parallelo{1})\ ...
            generators(zonotope(objCtrl_ff.R0));
    
    cext = [c;c;zeros(Opts.nx,1);zeros(Opts.nx,1)];
    Gext = [G;zeros(Opts.nx,size(G,2));G;Gbeta];
    
    R0 = zonotope(cext,Gext);

    % reachability settings
    params = Opts.ReachParams;
    options = Opts.ReachOpts;
    
    params.R0 = R0;
    params.tFinal = options.timeStep;
    
    params.paramInt = zeros(sys.nrOfParam,1);
    
    if ~isempty(Opts.V)
        params.R0 = cartProd(params.R0,Opts.V);
        params.U = cartProd(params.U,Opts.V);
    end
    
    % compute reachable set
    try
        reach(sys,params,options);
    catch
       return; 
    end
end

function initReachPoly(sys,objCtrl_ff,Opts)
% call CORA reach function once in advance so that the required tensors are
% calculated

    % construct initial set
    Uff_x = objCtrl_ff.Ctrl_x{1};  idv = Uff_x.id; o = zeros(Opts.nx,0);

    R0 = polyZonotope(Opts.R0);

    Uff = subs(Uff_x,R0,idv);
    R_xref = polyZonotope(center(R0),o,o,o,idv);
    
    if ~isempty(Opts.V)
        tmp = subs(-center(Uff_x) + Uff_x,polyZonotope(Opts.V),idv);
        Uff = Uff + zonotope(tmp);
    end

    R0 = stack(R0,R_xref,R0+(-center(R0)),project(Uff,1:Opts.nu));

    % reachability settings
    params = Opts.ReachParams;
    options = Opts.ReachOpts;
    
    params.R0 = R0;
    params.tFinal = options.timeStep;
    
    params.paramInt = zeros(sys.nrOfParam,1);
    
    if ~isempty(Opts.V)
        params.U = cartProd(params.U,Opts.V);
    else
        params.U = cartProd(params.U,zeros(Opts.nx,1));
    end
    
    % compute reachable set
    try
        reach(sys,params,options);
    catch
       return; 
    end
end

function [c,G] = linPolyCtrl2GenSpace(U)
% extract linear part from the input zonotope U

    c = center(U); 
    G = zeros(size(U.G,1),size(U.expMat,1));

    ind = find(sum(U.expMat,1) == 1);

    for i = 1:size(G,2)
        for j = 1:length(ind)
            if U.expMat(i,ind(j)) == 1
                G(:,i) = U.G(:,ind(j));
                break;
            end
        end
    end
end