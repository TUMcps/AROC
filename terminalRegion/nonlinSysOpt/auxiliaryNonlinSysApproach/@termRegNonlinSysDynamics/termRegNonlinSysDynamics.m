classdef termRegNonlinSysDynamics
    %TERMREGNONLINSYSDYNAMICS - helper class for storing dynamical system
    %       objects and auxiliary values used in the reachability analysis 
    %       of the abstraction.
    %
    % Syntax:
    %       obj = termRegNonlinSysDynamics(benchmark,sysType,solver,Opts,Param)
    %
    % Description:
    %       Helper class for storing dynamical system objects and auxiliary 
    %       values used in the reachability analysis of the abstraction. If
    %       a discrete-time dynamical system is considered, the
    %       continuous-time dynamics of the benchmark model are discretized
    %       during construction of an instance of this class.
    %
    % Input Arguments:
    %       -benchmark:     name of the considered benchmark model (see
    %                       "aroc/benchmarks/...")
    %       -sysType:       type of dynamical system, e.g., sampled-data 
    %                       or discrete-time system 
    %                       (class: termRegSysType)
    %       -solver:        instance of class 
    %                       computeTermRegNonlinSysOpt
    %       -Opts:          struct with the following fields
    %           .integrator:    (optional) integration scheme for time-
    %                           discretizing the dynamical system - only 
    %                           required if sysType == termRegSysType.DT
    %                           (class: discretizeDynamics)
    %           .timeStep:      sampling time 
    %                           (positive scalar)
    %           .nonlinSysTermRegAlg:   ('lin' / 'poly')
    %       -Param:         struct with the following fields
    %           .paramSet:      set of uncertain system parameters - only
    %                           required if sysType.isParamSystem == True
    %                           (class: interval)      
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               termRegNonlinSysDynamics
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
    %           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
    %           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
    %       [2] L. Schäfer and M. Althoff, "Computing Robust Control 
    %           Invariant Sets of Nonlinear Systems Using Polynomial 
    %           Controller Synthesis," American Control Conference, 2024, 
    %           pp. 4162-4169. 
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
    % Authors:      Lukas Schäfer
    % Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
    % Work Adress:  Technische Universitaet Muenchen
    % Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
    %               Embedded Systems, TU Muenchen
    %------------------------------------------------------------------

    
    properties(SetAccess = immutable, GetAccess = public)
        nx (1,1) {mustBeInteger}
        nu (1,1) {mustBeInteger}
        sysType termRegSysType = termRegSysType.SD % system type
        dt (1,1) double {mustBePositive} = 0.1 % sampling time
        dynamicsHandle (1,1) function_handle = @(x,u,w) 0 %function handle to the dynamics function of the open-loop system

        % nonlinear reachability analysis
        sys (1,1) contDynamics % dynamical system (class: nonlinearSysDT / nonlinearSys)
        reachOpts (1,1) struct = struct() % verification: algorithm settings for reachability analysis
        paramReachOpts (1,1) struct = struct() % parameterized reachability analysis (poly algorithm)
        reachParams (1,1) struct = struct() % verification: algorithm parameters for reachability analysis
        paramSet contSet = interval.empty(1)% interval containing the uncertain system parameters

        % parameterized reachability analysis
        Rfexp matZonotope % reachable set due to the dynamics evaluated at the equilibrium state and control input
	    AB_d matZonotope {mustBeUnderlyingType(AB_d,'matZonotope')} = 0 % horizontal concatenation of the system and input matrix of the linearized and discretized system
        RlinW (1,1) contSet % reachable set due to the set of disturbances (abstraction)
        Apower_dt_fact cell {mustBeUnderlyingType(Apower_dt_fact,'cell')} = {} % cell-array of (A*dt)^i/i!: A*dt, (A*dt)^2/2, (A*dt)^3/3!, ...
        nrTermsInputSolution (1,1) {mustBePositive,mustBeInteger} = 6 % number of terms in Apower_dt_fact
        
        % initial guess computation
        RlinW_dt_nom (1,1) contSet % nominal discrete-time reachable set due to the set of disturbances (abstraction)
    end
    
    methods
        function obj = termRegNonlinSysDynamics(benchmark,sysType,solver,Opts,Param)
            %TERMREGNONLINSYSDYNAMICS Construct an instance of this class
            
            obj.sysType = sysType;
            obj.dt = Opts.timeStep;
            obj.nx = solver.nx;
            obj.nu = solver.nu;

            % for discerte-time systems: set integration scheme for time-discretization
            if obj.sysType.isDiscreteTimeSystem
                integrator = Opts.integrator.getFunHandle;
            end

            % for systems with uncertain parameters: set of uncertain parameters
            if obj.sysType.isParamSystem
                if isfield(Param,'paramSet')
                    obj.paramSet = Param.paramSet;
                else
                    error('Set of uncertain parameters not provided!')
                end
            end

            % reachability settings
            % ... set tensor order
            if strcmp(Opts.nonlinSysTermRegAlg,'poly')
                tensorOrder= 3;
            else
                tensorOrder = 2;
            end
            % ... reachability settings for verification
            switch obj.sysType
                case termRegSysType.SD
                    obj.reachOpts.alg = 'poly-adaptive';
                otherwise
                    % use user-defined parameters to avoid expensive 
                    % additional tensor evaluation in adaptive algorithm
                    obj.reachOpts.tensorOrder = tensorOrder;
                    obj.reachOpts.zonotopeOrder = 100;
                    obj.reachOpts.errorOrder = 5;
                    obj.reachOpts.reductionTechnique = 'pca';                    
            end

            % parameters for reachability analysis
            obj.reachParams.tFinal = obj.dt;
            obj.reachParams.U = solver.W;

            % function handle to dynamics file 
            if sysType.isParamSystem
                str = ['dynamicsHandle = @(x,u,w,p)',benchmark,'(x,u,w,p);'];
            else
                str = ['dynamicsHandle = @(x,u,w)',benchmark,'(x,u,w);'];
            end
            eval(str);
            obj.dynamicsHandle = dynamicsHandle;

            % instantiate nonlinear system object
            switch obj.sysType
                case termRegSysType.DT                
                    % discretize dynamical system
                     obj.dynamicsHandle = @(x,u,w) integrator( ...
                            x,u,w,obj.dynamicsHandle,obj.dt); 
            
                    % instantiate nonlinearSysDT object
                    obj.sys = nonlinearSysDT(['terminalRegion_',benchmark], ...
                        @(x,w) [obj.dynamicsHandle(x(1:solver.nx),x(solver.nx+1:solver.nx+solver.nu),w); ...
                        zeros(solver.nu,1)], ...
                        obj.dt,solver.nx+solver.nu,solver.nw);
                case termRegSysType.paramDT
                    % discretize dynamical system
                    obj.dynamicsHandle = @(x,u,w,p) integrator( ...
                        x,u,w,p,obj.dynamicsHandle,obj.dt); 
            
                    % instantiate nonlinParamSys object
                    obj.sys = nonlinParamSys(['terminalRegion_',benchmark], ...
                        @(x,w,p) [obj.dynamicsHandle(x(1:solver.nx),x(solver.nx+1:solver.nx+solver.nu),w,p); ...
                        zeros(solver.nu,1)], ...
                        solver.nx+solver.nu,solver.nw,obj.paramSet.dim);
                case termRegSysType.SD
                    % instantiate nonlinearSys object
                    obj.sys = nonlinearSys(['terminalRegion_',benchmark], ...
                        @(x,w) [obj.dynamicsHandle(x(1:solver.nx),x(solver.nx+1:solver.nx+solver.nu),w); ...
                        zeros(solver.nu,1)], ...
                        solver.nx+solver.nu,solver.nw);         
            end

            % parameterized reachability analysis
            % ... dummy reachability settings for computing derivatives, etc.
            tmp_reachOpts.tensorOrder = tensorOrder;
            tmp_reachOpts.zonotopeOrder = 100;
            tmp_reachOpts.errorOrder = 5;
            tmp_reachOpts.reductionTechnique = 'pca';
            tmp_reachOpts.timeStep = obj.dt;
            if sysType.isParamSystem
                tmp_reachOpts.paramSet = obj.paramSet;
            end

            % ... compute derivatives
            derivatives(obj.sys,tmp_reachOpts); 
            switch obj.sysType
                case termRegSysType.DT
                    obj.Rfexp = zonotope(solver.xEq);
                    % linear abstraction
                    [jac_xu,jac_w] = obj.sys.jacobian( ...
                        [solver.xEq;solver.uEq],zeros(solver.nw,1));
                    obj.AB_d = matZonotope(jac_xu(1:solver.nx,:));
                    Ed = jac_w(1:solver.nx,:);
                    % ... reachable set due to the set of disturbances 
                    obj.RlinW = Ed*solver.W;
                    obj.RlinW_dt_nom = obj.RlinW;
                    
                    % abstraction error
                    obj.Apower_dt_fact = {[eye(solver.nx),zeros(solver.nx,solver.nu)]};
                    obj.nrTermsInputSolution = 1;

                case termRegSysType.paramDT
                    % linear abstraction
                    % ... nominal linearized system
                    pNom = center(obj.paramSet);
                    [jac_xu_nom,jac_w_nom] = obj.sys.jacobian_freeParam( ...
                        [solver.xEq;solver.uEq],zeros(solver.nw,1),pNom);
                    AB_d_nom = jac_xu_nom(1:solver.nx,:);
                    Ed_nom = jac_w_nom(1:solver.nx,:);
                    % ... expansion point
                    Rfexp = zonotope(obj.sys.mFile([solver.xEq;solver.uEq], ...
                        zeros(solver.nw,1),obj.paramSet));
                    Rfexp = project(Rfexp,1:solver.nx);
                    % ... center Rfexp at the nominal expansion point
                    obj.Rfexp = compact(zonotope(solver.xEq,[Rfexp.G,Rfexp.c-solver.xEq]));
                    % ... uncertain linearized system
                    [jac_xu,jac_w] = obj.sys.jacobian_freeParam( ...
                        [solver.xEq;solver.uEq],zeros(solver.nw,1),obj.paramSet);
                    jac_xu = intervalMatrix(interval(jac_xu.inf(1:solver.nx,:), ...
                        jac_xu.sup(1:solver.nx,:)));
                    AB_d = matZonotope(jac_xu);
                    % obj.Bd = jac_xu(1:solver.nx,solver.nx+(1:solver.nu));
                    if isa(jac_w,'interval')
                        jac_w = intervalMatrix(interval(jac_w.inf(1:solver.nx,:), ...
                            jac_w.sup(1:solver.nx,:)));
                        Ed = matZonotope(jac_w);
                    else
                        Ed = matZonotope(jac_w(1:solver.nx,:));
                    end
                    % ... center matrix zonotopes at the nominal system
                    obj.AB_d = matZonotope(AB_d_nom, cat(3,AB_d.G,AB_d.C-AB_d_nom));
                    % ... remove zero generators
                    AB_d_gens = [];
                    for ii = 1:obj.AB_d.numgens()
                        if any(obj.AB_d.G(:,:,ii) ~= 0,'all')
                            AB_d_gens = cat(3,AB_d_gens,obj.AB_d.G(:,:,ii));
                        end
                    end
                    obj.AB_d = matZonotope(obj.AB_d.C,AB_d_gens);

                    Ed = matZonotope(Ed_nom, cat(3,Ed.G,Ed.C-Ed_nom)); 
                    % ... reachable set due to the set of disturbances 
                    obj.RlinW = compact(Ed*solver.W);
                    obj.RlinW_dt_nom = compact(Ed.C*solver.W);
                    
                    % abstraction error
                    obj.Apower_dt_fact = {eye(solver.nx)};
                    obj.nrTermsInputSolution = 1;

                    
                case termRegSysType.SD
                    % linear abstraction
                    % linearize dynamical system
                    tmp_reachOpts.linearizationPoint = [solver.xEq;solver.uEq];
                    tmp_linReachOpts = tmp_reachOpts;
                    tmp_linReachOpts.alg = 'lin';
                    % tmp_reachOpts.taylorOrder = 6;
                    tmp_linReachParams = obj.reachParams;
                    tmp_linReachParams.uTrans = zeros(solver.nw,1);
                    [~,linSys,linParams,~] = linearize(obj.sys,[],tmp_linReachParams,tmp_linReachOpts);
                    % ... discretize linearized system
                    f0 = obj.sys.mFile([solver.xEq;solver.uEq], ...
                        zeros(solver.nw,1));
                    Rfexp = particularSolution_constant( ...
                        linSys,f0,obj.dt,obj.nrTermsInputSolution);
                    obj.Rfexp = project(zonotope(Rfexp),1:solver.nx);
                    [~] = homogeneousSolution(linSys, ...
                        zonotope(zeros(solver.nx+solver.nu,1),eye(solver.nx+solver.nu)), ...
                        tmp_linReachParams.tFinal,obj.nrTermsInputSolution);
                    jac_xu = linSys.taylor.eAdt{1};
                    obj.AB_d = jac_xu(1:solver.nx,:);
                    % ... reachable set due to the set of disturbances 
                    R_deltaW = particularSolution_timeVarying( ...
                        linSys,linParams.U,obj.dt,obj.nrTermsInputSolution);
                    [R_centerW,~] = particularSolution_constant( ...
                        linSys,linParams.uTrans,obj.dt,obj.nrTermsInputSolution);
                    obj.RlinW = project(compact(R_centerW + R_deltaW),1:solver.nx);
                    % ... discrete-time approximation of the reachable set
                    % due to the set of disturbances (for initial guess
                    % computation)
                    temp = integral(@(t)expm(linSys.A(1:solver.nx,1:solver.nx)*t),0,obj.dt,'ArrayValued',true);   
                    [~,jac_w] = obj.sys.jacobian(tmp_linReachOpts.linearizationPoint,zeros(solver.nw,1));
                    Ec = jac_w(1:solver.nx,:);
                    obj.RlinW_dt_nom = temp*Ec*solver.W;

                    % abstraction error
                    % remove matrices full of zeros
                    obj.Apower_dt_fact{1} = linSys.taylor.dtoverfac{1}(1)*[eye(solver.nx), zeros(solver.nx,solver.nu)];
                    for ii = 1:length(linSys.taylor.Apower)
                        if any(linSys.taylor.Apower{ii}(1:solver.nx,1:solver.nx),'all')
                            obj.nrTermsInputSolution = ii+1;
                            obj.Apower_dt_fact{ii+1} = ...
                                linSys.taylor.dtoverfac{1}(ii+1)*linSys.taylor.Apower{ii}(1:solver.nx,:);
                        else
                            break
                        end
                    end

                    % ... poly-algorithm [2] only: add missing settings
                    % for parameterized reachability analysis
                    if strcmp(Opts.nonlinSysTermRegAlg,'poly')
                        tmp_reachOpts.approxDepOnly = true; % neglect abstraction error
                        tmp_reachOpts.alg = 'poly';
                        tmp_reachOpts.timeStep = obj.dt;
                        tmp_reachOpts.taylorTerms = length(obj.Apower_dt_fact);
                        % tmp_reachOpts.factor = linSys.taylor.dtoverfac{1}(1:tmp_reachOpts.taylorTerms+1);
                    end
            end

            % ... poly-algorithm [2] only: store settings for parameterized
            % reachability analysis
            if strcmp(Opts.nonlinSysTermRegAlg,'poly')
                obj.paramReachOpts = tmp_reachOpts;
            end
        end
    
        function RabsErr = computeReachSetAbsErr(obj,absErr_c,absErr_G)
            % Returns an underapproximation of the reachable set due to the 
            % set of abstraction errors zonotope(absErr_c,absErr_G)
            % Input arguments must be given in the augmented state space!
            
            RabsErr.c = zeros(obj.nx,1);
            RabsErr.G = [];
            for ii = 1:obj.nrTermsInputSolution
                RabsErr.c = RabsErr.c + obj.Apower_dt_fact{ii}*absErr_c;
                RabsErr.G = [RabsErr.G,obj.Apower_dt_fact{ii}*absErr_G];
            end
        end
    
    end
end

