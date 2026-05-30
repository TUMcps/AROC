classdef computeTermRegNonlinSysPolyApproach < computeTermRegNonlinSysOpt
    %COMPUTETERMREGNONLINSYSPOLYAPPROACH - solver class for the
    %       polynomialization-based approach for computing terminal 
    %       regions of nonlinear systems.
    %
    % Syntax:
    %       obj = computeTermRegNonlinSysPolyApproach(Param,Opts)
    %
    % Description:
    %       This class provides all methods required for formulating and 
    %       solving the polynomial approximations of the terminal region 
    %       optimization problem in [1, Eq. 6] as well as the methods for
    %       verifying correctness of its solution.
    %
    % Input Arguments:
    %
    %       -Param:         a struct containing the benchmark parameters
    %           .X:         set of admissible states
    %                       (class: interval/zonotope/polytope)
    %           .U:         set of admissible control inputs
    %                       (class: interval/zonotope/polytope)
    %           .W:         set of disturbances
    %                       (class: zonotope)
    %
    %       -Opts:          a struct containing the algorithm settings
    %           .xEq:                   equlibrium state to be stabilized
    %                                   (double vector)
    %           .uEq:                   equilibrium control input
    %                                   (double vector)
    %           .termRegOrder:          (optional) order of the zonotope 
    %                                   representing the terminal region
    %                                   (positive scalar, >= 1)
    %           .verbose:               if set to true, the algorithm prints 
    %                                   information about each iteration to the 
    %                                   command window. If set to false, no 
    %                                   iterative output is shown. 
    %           .solverSettings:    (optional) struct with the following
    %                               optional fields (see superclass
    %                               constructor for shared fields)
    %               -.paramReachSetOrder:   order of the zonotope
    %                                       enclosing the parameterized 
    %                                       reachable set for optimization
    %                                       (positive integer)
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               computeTermRegNonlinSysPolyApproach
    %
    % See Also:
    %       -
    % 
    % References:
    %       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
    %           Sets of Nonlinear Systems Using Polynomial Controller
    %           Synthesis," American Control Conference, 2024, pp. 4162-4169.
    %       [2] V. Gaßmann and M. Althoff, "Verified Polynomial Controller 
    %           Synthesis for Disturbed Nonlinear Systems, IFAC-PapersOnLine,
    %           Volume 54, Issue 5, 2021, pages 85-90.
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
    % Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
    %               Embedded Systems, TU Muenchen
    %------------------------------------------------------------------
     

    properties(SetAccess = immutable, GetAccess=public)
        % algorithm settings
        paramReachSetOrder (1,1) {mustBeGreaterThanOrEqual(paramReachSetOrder,1), ...
            mustBeInteger} = 25 % order of the (zonotope enclosing the) parameterized approximation of the reachable set for optimization

        id struct % stores the polynomial zonotope ids representing the optimization variables for parameterized reachability analysis
        ctrl struct % stores the parameters of the controller templates like the exponent matrix (see [1, Eq. 14])
    end

    properties(SetAccess=private, GetAccess=public)
        pZu_vp % controller template - see [1, Eq. 15] with v = lambda_S
    end

    methods
        function obj = computeTermRegNonlinSysPolyApproach(Param,Opts)
            %COMPUTETERMREGNONLINSYSPOLYAPPROACH Construct an instance of this class
            
            % check optional input arguments
            if isfield(Opts,'solverSettings')
                solverSettings = Opts.solverSettings;
            else
                solverSettings = struct();
            end
            % call superclass constructor
            obj = obj@computeTermRegNonlinSysOpt( ...
                Param.X,Param.U,Param.W, ...
                Opts.xEq,Opts.uEq,Opts.termRegOrder,Opts.verbose,solverSettings);

            % copy class-specific algortihm settings
            if isfield(solverSettings,'paramReachSetOrder')
                obj.paramReachSetOrder = solverSettings.paramReachSetOrder;
            end 

            % prepare controller template
            % ... for now: controller order set to one!
            ctrlOrder = 1;
            ctrl.dim = obj.nx*obj.termRegOrder;
            % ctrl_expMat = generateExpMat(ctrl.dim,ctrlOrder);
            % ctrl_expMat_unique = unique(ctrl_expMat','rows','stable')';
            % if size(ctrl_expMat_unique,2)<size(ctrl_expMat,2)
            %     disp('"eMCtrl" contains duplicate vectors - removing...');
            % end
            ctrl_expMat_unique = [zeros(ctrl.dim,1),eye(ctrl.dim)];
            ctrl.expMat = ctrl_expMat_unique;
            ctrl.np_s = size(ctrl.expMat,2);
            ctrl.np = obj.nu*ctrl.np_s;
            ctrl.Np = ctrl.np;

            obj.ctrl = ctrl;

            % ---- pre-compute polyZonotope ids --------------------------

            % ids of candidate terminal region
            id.termReg = (1:obj.nx*obj.termRegOrder)';
            idMax = obj.termRegOrder*obj.nx;
            
            % auxiliary ids: order reduction of reachable set
            id.red = (idMax + (1:obj.nx))';
            idMax = idMax + obj.nx;

            % ids for abstraction error
            id.absErr = (idMax + (1:obj.nx))'; 
            idMax = idMax + obj.nx;
            
            % auxiliary ids: pw-linear approximation of the abstraction
            % error
            id.absErr_aux = (idMax + (1:2))';
            idMax = idMax + 2;

            % (optional) ids of cost set
            if obj.flagCostExact
                id.cost = id.termReg;
            else
                nGen = floor(obj.nx*obj.costSetOrder);
                id.cost = reshape(idMax + (1:nGen),[],1);
                idMax = idMax + length(id.cost);
            end

            id.idMaxAux = idMax;

            % ids forcontroller synthesis
            % ... for scalar input
            id.p_s = -(1:obj.ctrl.np_s)';
            % ... for all inputs
            id.p = -((1:obj.ctrl.Np))';
            % ... ids of p(i,1) - control input for the center
            id.pc = (id.p(1):-obj.ctrl.np_s:id.p(end))';
            idMin = -obj.ctrl.Np;
            
            % translation of the center of terminal region
            id.delta_c  = idMin-(1:obj.nx)';
            idMin = idMin - obj.nx;
                       
            % auxiliary ids for setting up the controller template
            % ... initial set
            id.b = idMin - (1:ctrl.dim)';
            idMin = idMin - ctrl.dim;
            % ... center of initial set
            id.c = idMin - (1:ctrl.dim)';

            % ids of scalar optimization variables:
            % -> controller parameters and translation of center
            id.var_scalar = sort([id.p; id.delta_c ],'descend');

            % ids of (interval) scaling optimization variables
            % -> generator scaling factors of terminal region, scaling 
            % factors of order reduction set + Lagrange remainder
            id.var_scaling = sort(vertcat(id.termReg,id.red,id.absErr));

            % collect reachable set optimization variables
            id.var_optReach = [id.var_scalar;id.var_scaling];

            obj.id = id;

            % ---- controller --------------------------------------------
            % based on [2], see also polynomialControl

            % setup dummy opts
            tmp_opts = obj.ctrl;
            tmp_opts.nx = ctrl.dim;
            tmp_opts.nu = obj.nu;
            tmp_opts.Ninter = 1;
            tmp_opts.eMCtrl = obj.ctrl.expMat;
            % tmp_opts.np_s = obj.ctrl.np_s;
            % tmp_opts.Np = opts.ctrl.Np;
            tmp_opts.idw = obj.id.b;
            tmp_opts.idp_s = obj.id.p_s;
            tmp_opts.idp = obj.id.p;
            tmp_opts.idc = obj.id.c;
            
            % mapping from interpolation factors of the initial set (b), 
            % center shift (due to parallelotope enclosure, see [2]) (c), 
            % and the controller parameters (p) to the interpolation 
            % factors of the set of admissible control inputs (a) 
            % [2, Eq. 4]
            [~,pZa_bcp] = setupControlTemplates(tmp_opts);
            % ... resolve center shift (initial set = terminal region, which
            % is represented as a zonotope)
            dcb = zeros(obj.ctrl.dim,1);
            pZa_bp = resolve(pZa_bcp,dcb,obj.id.c);
            % ... express b as function of the termial region interpolation
            % factors (lambda_S in [1, Eq. 14])
            pZb_v = polyZonotope( ...
                zeros(ctrl.dim,1),eye(ctrl.dim),[], ...
                eye(ctrl.dim),obj.id.termReg);
            pZa_vp = subs(pZa_bp,pZb_v,obj.id.b);
            % ... map directly to control inputs - see [1, Eq. 15]
            obj.pZu_vp = generators(obj.Z_U)*pZa_vp + center(obj.Z_U);

        end

        function obj = updateCtrlTemplate(obj,s)
            % Since scaling of the terminal region also affects the control
            % law (see [1, Eq. (15)]), the controller template must be
            % update for the next iteration.
            %
            % Input Argument:
            %       -s:     (optimal) generator scaling factors
            %               (double vector)
            %
            % Output Argument:
            %       -obj:   instance of this class with scaled controller
            %               template
            %
            % ------------------------------------------------------------

            pZ_sOpt = polyZonotope(zeros(length(obj.id.termReg),1),diag(s),[], ...
                eye(length(obj.id.termReg)),obj.id.termReg);

            obj.pZu_vp = subs(obj.pZu_vp,pZ_sOpt,obj.id.termReg);
        end

        function obj = updateCostSet(obj,s)
            % Given the optimized set of scaling factors of the set whose
            % volume is optimized, these sets generators are scaled.
            %
            % Input Argument:
            %       s:  (optimal) generator scaling factors
            %           (double vector)
            %
            % Output Argument:
            %       -obj:   instance of this class with scaled cost set
            %
            % ------------------------------------------------------------
            
            % polyZonotope-representation requires scaled set for
            % substitution into costSet
            pZ_sOpt = polyZonotope(zeros(length(obj.id.cost),1),diag(s),[], ...
                eye(length(obj.id.cost)),obj.id.cost);

            obj.costSet = subs(obj.costSet,pZ_sOpt,obj.id.cost);

        end
    end

    methods(Static)
        % parameterized partial zonotope over-approximation of polynomial
        % zonotopes 
        partZono = paramPartZonotope(pZ,idv,idp);

        % initial guess for dual variables
        [lam_g,lam_x] = computeDualInitialGuess(x0,cost_handle,con_handle,idxVars,idxCon,lbx,ubx);

        % evaluation of polyZonotopes for a given value of the identifiers
        res = evalPZfuncCasadi(pZ,x,id);
    end
end
