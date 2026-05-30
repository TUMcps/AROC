classdef computeTermRegNonlinSysOpt
    %COMPUTETERMREGNONLINSYSOPT - abstract superclass for 
    %       optimization-based approaches for computing terminal regions of 
    %       nonlinear systems. 
    %
    % Syntax:
    %       obj = computeTermRegNonlinSysOpt(X,U,W,xEq,uEq,termRegOrder)
    %       obj = computeTermRegNonlinSysOpt(X,U,W,xEq,uEq,termRegOrder,solverSettings)
    %
    % Description:
    %       This class and its childclasses provide all methods required
    %       for formulating and solving the convex [1] or polynomial [2]
    %       approximations of the terminal region optimization problem in
    %       [2, Eq. (6)] as well as the methods for verifying correctness 
    %       of the candidate solutions. 
    %
    % Input Arguments:
    %           -X:                 set of admissible states 
    %                               (class: interval / polytope / zonotope)
    %           -U:                 set of admissible control inputs
    %                               (class: interval / polytope / zonotope)
    %           -W:                 set of uncertain disturbances 
    %                               (class: zonotope)
    %           -xEq:               equilibrium state
    %                               (array of dimension [X.dim,1])
    %           -uEq:               equilibrium control input
    %                               (array of dimension [U.dim,1])
    %           -termRegOrder:      order of the zonotope representing the 
    %                               terminal region 
    %                               (positive integer) 
    %           -verbose:           if set to true, the algorithm prints 
    %                               information about each iteration to the 
    %                               command window. If set to false, no 
    %                               iterative output is shown. 
    %           -solverSettings:    (optional) struct with the following
    %                               optional fields
    %               -.penaltyWeight:        penalty weight for soft 
    %                                       constraints, see [1, Remark 4]
    %                                       (positive scalar)
    %               -.convergenceTolerance: convergence tolerance, see 
    %                                       [1, Eq. 18]
    %                                       (positive scalar)
    %               -.constraintTolerance:  constraint tolerance
    %                                       (positive scalar)
    %               -.maxIter:              max. number of iterations
    %                                       (positive integer)
    %               -.maxCostTerms:         maximum admissible number of 
    %                                       addends in the cost function, 
    %                                       see [1, Remark 2]
    %                                       (positive interger)
    %               -.tr_maxRelErr:         max. relative approximation
    %                                       error (see [1, App. II])
    %                                       (0 < tr_malRelErr < 1)
    %               -.tr_shrink:            scaling factor for shrinking
    %                                       the trust region (see 
    %                                       [1, App. II])
    %                                       (0 < tr_shrink < 1)
    %               -.tr_grow:              scaling factor for enlarging
    %                                       the trust region (see 
    %                                       [1, App. II])
    %                                       (1 < tr_grow)
    %
    % Output Arguments:
    %       -obj:   generated instance of the class 
    %               computeTermRegNonlinSysOpt
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
    % Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
    %               Embedded Systems, TU Muenchen
    %------------------------------------------------------------------
    
    properties(SetAccess = immutable, GetAccess = public)
        % algorithm parameters and settings
        penaltyWeight (1,1) double {mustBePositive} = 1e4 % penalty weight for soft constraints, see [1, Remark 4]
        convergenceTolerance (1,1) double {mustBePositive} = 5e-3 % convergence tolerance, see [1, Eq. 18]
        constraintTolerance (1,1) double {mustBePositive} = 1e-8 % constraint tolerance
        maxIter (1,1) {mustBePositive, mustBeInteger} = 25; % maxmimum number of iterations
        maxCostTerms (1,1) {mustBePositive, mustBeInteger} = 1500; % maximum admissible number of addends in the cost function, see [1, Remark 2]
        costSetOrder (1,1) double % order of the set whose size is optimized
        flagCostExact (1,1) logical = true; % false if the number of addends in the cost function exceeds maxCostTerms, true otherwise
        
        % solution parameters
        termRegOrder (1,1) {mustBePositive, mustBeInteger} = 5 % order of the zonotope representing the terminal region

        % system-related parameters and settings
        xEq double {mustBeVector} = 0 % equilibrium state
        uEq double {mustBeVector} = 0 % equilibrium control input
        nx (1,1) % dimension of the state space 
        nu (1,1) % dimension of the control input space
        nw (1,1) % dimension of the disturbance space

        % constraint sets
        H_X (1,1) contSet % set of admissible states - specification (class: polytope)
        H_U (1,1) contSet % half-space representation of U
        Z_U (1,1) contSet % generator representation of U
        W (1,1) contSet % set of disturbances

        % print to command window
        verbose (1,1) logical = 0

        % parameters for updating the trust region, see [1, Appendix II] 
        tr_maxRelErr (1,1) double {mustBePositive} = 0.1 % max. relative approximation  error
        tr_shrink (1,1) double {mustBePositive, mustBeLessThan(tr_shrink,1)} = 1/2.0 % scaling factor for shrinking the trust region (see 
        tr_grow (1,1) double {mustBePositive, mustBeGreaterThan(tr_grow,1)} = 3.2 % scaling factor for enlarging the trust region
    end

    properties(SetAccess = protected, GetAccess = public)
        % constraint sets
        H_X_con (1,1) contSet % (tightened) set of admissible states - constraints (class: polytope)
        
        % cost function parameters
        costCoefficients double {mustBeVector,mustBeNonnegative} = 0 % vector storing the coefficients of the cost function
        costExponents {mustBeMatrix, mustBeInteger} = 0 % matrix storing the exponents of the cost function monomials
        costSet (1,1) contSet % set whose volume is optimized, see [1, Remark 2]

        argAbsErr % state dimensions that influence the abstraction error
    end

    properties(SetAccess = public, GetAccess = public)
        tr_set (1,1) contSet % trust region
    end

   properties (Dependent)
      costSetVolume % volume of costSet
   end
    
    methods

        % class constructor
        function obj = computeTermRegNonlinSysOpt(X,U,W,xEq,uEq,termRegOrder,verbose,varargin)

            obj.termRegOrder = termRegOrder;
            obj.xEq = xEq;
            obj.uEq = uEq;

            obj.verbose = verbose;
            
            % check consistency of equilibrium state and set of admissible
            % states
            % -> dimensions must agree
            assert(length(xEq) == X.dim, ...
                ['Inconsistent dimensions of equilibrium state and set ' ...
                'of admissible states!'])
            % -> xEq must be contained in the interior of X
            xEq_set = zonotope(xEq, 1e-3*eye(length(xEq)));
            assert(contains(X,xEq_set), ...
                ['Equilibrium state is not contained in ' ...
                'the interior of the set of ' ...
                'admissible control inputs!'])
            obj.nx = X.dim;
            obj.argAbsErr = 1:obj.nx;

            % check type of set of admissible states
            if ~(isa(X,'zonotope') || isa(X,'polytope') || ...
                    isa(X,'interval'))
                error(['Invalid class of set of admissible states. ' ...
                    'Accepted classes are ..."zonotope", "polytope", ' ...
                    'and "interval".'])
            end
            % convert to half-space representation
            obj.H_X = polytope(X);
            obj.H_X_con = obj.H_X;

            % check consistency of equilibrium control input and set of
            % admissible control inputs
            % -> dimensions must agree
            assert(length(uEq) == U.dim, ...
                ['Inconsistent dimensions of equilibrium control input '...
                'and set of admissible control inputs!'])
            % -> uEq must be contained in the interior of U
            uEq_set = zonotope(uEq,1e-3*eye(length(uEq)));
            assert(contains(U,uEq_set), ...
                ['Equilibrium control input is not contained in ' ...
                'the interior of the set ' ...
                'of admissible control inputs!'])
            obj.nu = U.dim;

            % check type of set of admissible control inputs
            switch class(U)
                case 'zonotope'
                    obj.Z_U = U;
                    obj.H_U = polytope(U);
                case 'polytope'
                    obj.H_U = U;
                    obj.Z_U = zonotope(interval(U));
                case 'interval'
                    obj.H_U = polytope(U);
                    obj.Z_U = zonotope(U);
                otherwise
                    error(['Invalid class of set of admissible ' ...
                        'control inputs. Accepted classes are ...' ...
                        '"zonotope", "polytope", and "interval".'])
            end
            
            % check type of set of disturbances
            if ~isa(W,'zonotope') && ~isa(W,'interval')
                error(['Invalid class of set of disturbances. ' ...
                    'W must be of class interval or zonotope.'])
            elseif ~contains(W,zeros(W.dim,1))
                error('Set of disturbances does not contain the origin.')
            else
                obj.W = zonotope(W);
                obj.nw = W.dim;
            end

            % copy algorithm settings from optional input argument
            if ~isempty(varargin) && isstruct(varargin{1})
                solverSettings = varargin{1};
                if isfield(solverSettings,'penaltyWeight')
                    obj.penaltyWeight = solverSettings.penaltyWeight;
                end
                if isfield(solverSettings,'convergenceTolerance')
                    obj.convergenceTolerance = solverSettings.convergenceTolerance;
                end
                if isfield(solverSettings,'constraintTolerance')
                    obj.constraintTolerance = solverSettings.constraintTolerance;
                end
                if isfield(solverSettings,'maxIter')
                    obj.maxIter = solverSettings.maxIter;
                end
                if isfield(solverSettings,'maxCostTerms')
                    obj.maxCostTerms = solverSettings.maxCostTerms;
                end
                if isfield(solverSettings,'tr_maxRelErr')
                    obj.tr_maxRelErr = solverSettings.tr_maxRelErr;
                end
                if isfield(solverSettings,'tr_shrink')
                    obj.tr_shrink = solverSettings.tr_shrink;
                end
                if isfield(solverSettings,'tr_boundShrink')
                    obj.tr_grow = solverSettings.tr_boundShrink;
                end
            end

            % check whether the size of the terminal region itself or an
            % under-approximation thereof is chosen as the cost function 
            % (see [1, Remark 2]) 
            try 
                comb = combinator(obj.termRegOrder*obj.nx,obj.nx,'c');
                if size(comb,1) > obj.maxCostTerms
                    obj.flagCostExact = false;
                else
                    obj.flagCostExact = true;
                end
            catch
                % error while computing combinations, most likely out of 
                % memory -> consider reduced terminal region for the cost
                obj.flagCostExact = false;
            end
            % set order of the cost set
            if obj.flagCostExact 
                obj.costSetOrder = termRegOrder;
            else
                tmp = cell(obj.nx,1);
                for ii = 1:obj.nx
                   tmp{ii} = @(x) (x-1)*obj.nx + ii; 
                end
                fun = @(x) prod(cellfun(@(F) F(x), tmp)) ...
                    - obj.maxCostTerms*factorial(obj.nx);
                costSetOrder = max(fsolve(fun, obj.termRegOrder, ...
                    optimset('Diagnostics','off', 'Display','off')),1);  
                % ensure integer number of generators
                nGen = floor(costSetOrder*obj.nx);
                obj.costSetOrder = (nGen+1e-6)/obj.nx;
            end
    
        end

        function obj = tightenStateConstraintSet(obj,b_tight)
            % update set of admissible states (required for verification of
            % the terminal region of sampled-data systems)
            % Previuous set of admissible states: {x: A*x <= b}
            % Updated set of admissible states: {x: A*x <= b-b_tight}
            assert(all(b_tight >= 0) ...
                && length(obj.H_X.b) == length(b_tight))
            X_tight = polytope(obj.H_X.A,obj.H_X_con.b-b_tight);
            assert(contains(X_tight,obj.xEq))

            obj.H_X_con = X_tight;
        end

        function obj = set.tr_set(obj,tr)
            % check wether tr is represented as a zonotope, centered at
            % the origin, and has dimension obj.nx
            assert(isa(tr,'zonotope') && all(tr.c == 0) && tr.dim == obj.nx ...
                && isdiag(tr.G) && all(diag(tr.G) >= 0), ...
                'Invalid trust region set.')
            obj.tr_set = tr;
        end

        function vol = get.costSetVolume(obj)
            % returns the size of the current cost set
            vol = (2^obj.nx)*sum(obj.costCoefficients);
        end
    end
end
