function Opts = checkOpts(Opts,name,Param,nx,nu,varargin)
% CHECKOPTS - check if the specified settings take correct values
%
% Syntax:
%       Opts = CHECKOPTS(Opts,name,Param,nx,nu)
%       Opts = CHECKOPTS(Opts,name,Param,nx,nu,isLin)
%
% Description:
%       Checks if the specified algorithm settings take valid values and 
%       assign default values to settings that are not specified.
%
% Input Arguments:
%
%       -Opts:      a structure containing the algorithm settings
%       -name:      name of the control algorithm, e.g.
%                   'generatorSpaceControl', etc.
%       -Param:     a structure containing the benchmark parameters
%       -nx:        number of system states
%       -nu:        number of inputs
%       -isLin:     flag specifying if the model is linear (isLin = 1) or
%                   nonlinear (isLin = 0)
%
% Output Arguments:
%
%       -Opts:      adapted structure of algorithm settings
%
% See Also:
%       generatorSpaceControl, convexInterpolationControl
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

    % checks for the different control algorithms
    switch name
             
        case 'convexInterpolationControl'
            
            % number of time steps Opts.N
            Opts = checkNumTimeSteps(Opts,10);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkNumInterTimeSteps(Opts,4);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkReachSteps(Opts,20);
            
            % state weighting matrix Opts.Q
            Opts = checkStateWeightingMatrix(Opts,'Q',nx);
            
            % input weighting matrix Opts.R
            Opts = checkInputWeightingMatrix(Opts,'R',nu);
            
            % control law Opts.controller
            Opts = checkController(Opts,'linear');
            
            % parallel computing Opts.parallel
            Opts = checkParallel(Opts,0);
            
            % control law approximation settings Opts.approx
            [Opts,redFields] = checkApprox(Opts,'scaled',0.5);
            
            % reference trajectory Opts.refTraj
            [Opts,temp] = checkReferenceTrajectory(Opts,Param,nx,nu,0);
            redFields = [redFields,temp];
            
            % extended optimization horizon
            [Opts,temp] = checkExtendedHorizon(Opts);
            redFields = [redFields,temp];
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 20;         
            Def.zonotopeOrder = 100;     
            Def.intermediateOrder = 50;
            
            if strcmp(Opts.controller,'linear')
                Def.errorOrder = 5;
                Def.alg =  'lin';
                Def.tensorOrder = 2;
            else
                Def.errorOrder = 30;
                Def.alg =  'poly';
                Def.tensorOrder = 3;
            end
            
            [Opts,temp] = checkCoraSettings(Opts,Def);
            redFields = [redFields,temp];           
            
            % polynomial zonotope restructure settings Opts.polyZono
            [Opts,temp] = checkPolyZono(Opts,inf,10,20);
            redFields = [redFields,temp];
            
            % valid settings for this algorithm
            validFields = {'N','Ninter','reachSteps','Q','R', ...
                           'controller','approx','polyZono','parallel', ...
                           'refTraj','extHorizon','cora'};
                  
            
        case 'generatorSpaceControl'
            
            % number of time steps Opts.N
            Opts = checkNumTimeSteps(Opts,10);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkNumInterTimeSteps(Opts,4);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkReachSteps(Opts,10);
            
            % state weighting matrix Opts.Q
            Opts = checkStateWeightingMatrix(Opts,'Q',nx);
            
            % input weighting matrix Opts.R
            Opts = checkInputWeightingMatrix(Opts,'R',nu);
            
            % use inputs from reference trajectory for the center
            Opts = checkRefInput(Opts,false);
            
            % reference trajectory Opts.refTraj
            [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,1);
            
            % extended optimization horizon
            [Opts,temp] = checkExtendedHorizon(Opts);
            redFields = [redFields,temp];
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 20;         
            Def.zonotopeOrder = 30;     
            Def.intermediateOrder = 20;
            Def.errorOrder = 5;
            Def.alg =  'lin';
            Def.tensorOrder = 2;
            
            [Opts,temp] = checkCoraSettings(Opts,Def);
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','Ninter','reachSteps','Q','R', ...
                           'refInput','refTraj','extHorizon','cora'};
                       
            
        case 'optimizationBasedControl'
            
            % number of time steps Opts.N
            Opts = checkNumTimeSteps(Opts,5);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkReachSteps(Opts,10);
            
            % number of final reachability steps Opts.reachStepsFin
            Opts = checkReachStepsFin(Opts,100);
            
            % scaling factor between upper and lower bound Opts.bound
            Opts = checkBound(Opts,1000);
            
            % maximum number of iterations for fmincon Opts.maxIter
            Opts = checkMaxIter(Opts,15);
            
            % reference trajectory Opts.refTraj
            [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,0);
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 50;     
            Def.intermediateOrder = 30;
            Def.errorOrder = 5;
            Def.alg =  'lin';
            Def.linAlg = 'standard';
            Def.tensorOrder = 2;
            
            if nargin > 5 && varargin{1} == 0
                [Opts,temp] = checkCoraSettings(Opts,Def);
            else
                [Opts,temp] = checkCoraSettingsLin(Opts,Def);
            end
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','reachSteps','reachStepsFin','bound', ...
                           'maxIter','refTraj','cora'};
        
            
        case 'reachsetMPC'
            
            % final time for optimization Opts.tPredict
            checkOptimizationTime(Opts);
            
            % number of time steps Opts.N
            Opts = checkNumTimeSteps(Opts,10);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkReachSteps(Opts,10);
            
            % set of tightend input constraints Opts.U_
            checkTightendInputSet(Opts,Param,nu);
            
            % terminal region Opts.termReg
            checkTerminalRegion(Opts,Param,nx);
            
            % state weighting matrix Opts.Q
            Opts = checkStateWeightingMatrix(Opts,'Q',nx);
            
            % input weighting matrix Opts.R
            Opts = checkInputWeightingMatrix(Opts,'R',nu);
            
            % state weighting matrix for tracking controller Opts.Qlqr
            Opts = checkStateWeightingMatrixLQR(Opts,nx);
            
            % input weighting matrix for tracking controller Opts.Rlqr
            Opts = checkInputWeightingMatrixLQR(Opts,nu);
            
            % real-time mode Opts.realTime
            Opts = checkRealTime(Opts,true);
            
            % allocated computation time Opts.tComp
            checkAllocatedComputationTime(Opts);
            
            % contraction rate Opts.alpha
            Opts = checkContractionRate(Opts,0.1);
            
            % maximum number of iterations for optimal control Opts.maxIter
            Opts = checkMaxIter(Opts,10);
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 5;     
            Def.intermediateOrder = 5;
            Def.errorOrder = 3;
            Def.alg =  'lin';
            Def.tensorOrder = 2;
            
            [Opts,redFields] = checkCoraSettings(Opts,Def);          
            
            % valid settings for this algorithm
            validFields = {'tOpt','N','reachSteps','Q','R','Qlqr', ...
                           'Rlqr','U_','termReg','realTime','tComp' ...
                           'alpha','maxIter','cora'};
            
        otherwise
            error('Wrong value for input argument "name"!');
    end
    
    % warnings if additional fields are provided that are not required
    temp = getRedundantFields(Opts,[],validFields);
    redFields = [redFields,temp];
    
    if ~isempty(redFields)
        text = redFields{1};
        for i = 2:length(redFields)
           text = [text,', ',redFields{i}]; 
        end
        warning(['The following fields of struct "Opts" are redundant: ',text]);
    end
end


% Auxiliary Functions -----------------------------------------------------

function Opts = checkNumTimeSteps(Opts,defVal)
% check if the number of time steps N takes a correct value

    if ~isfield(Opts,'N')
       Opts.N = defVal; 
    elseif ~isnumeric(Opts.N) || ~isscalar(Opts.N) || ...
           mod(Opts.N,1) ~= 0 || Opts.N < 1 
       error('Wrong value for "Opts.N"! Has to be an integer >= 1!');
    end
end

function Opts = checkNumInterTimeSteps(Opts,defVal)
% check if the number of intermediate time steps Ninter takes a correct 
% value

    if ~isfield(Opts,'Ninter')
       Opts.Ninter = defVal; 
    elseif ~isnumeric(Opts.Ninter) || ~isscalar(Opts.Ninter) || ...
           mod(Opts.Ninter,1) ~= 0 || Opts.Ninter < 1 
       error('Wrong value for "Opts.Ninter"! Has to be an integer >= 1!');
    end
end

function Opts = checkReachSteps(Opts,defVal)
% check if the number of reachability steps Opts.reachSteps takes a correct 
% value

    if ~isfield(Opts,'reachSteps')
       Opts.reachSteps = defVal; 
    elseif ~isnumeric(Opts.reachSteps) || ~isscalar(Opts.reachSteps) || ...
           mod(Opts.reachSteps,1) ~= 0 || Opts.reachSteps < 1 
       error('Wrong value for "Opts.reachSteps"! Has to be an integer >= 1!');
    end
end

function Opts = checkReachStepsFin(Opts,defVal)
% check if the number of final reachability steps Opts.reachStepsFin takes 
% a correct value

    if ~isfield(Opts,'reachStepsFin')
       Opts.reachStepsFin = defVal; 
    elseif ~isnumeric(Opts.reachStepsFin) || ~isscalar(Opts.reachStepsFin) || ...
           mod(Opts.reachStepsFin,1) ~= 0 || Opts.reachStepsFin < 1 
       error('Wrong value for "Opts.reachStepsFin"! Has to be an integer >= 1!');
    end
end

function Opts = checkBound(Opts,defVal)
% check if the scaling factor between the upper and the lower bound 
% Opts.bound takes a correct value

    if ~isfield(Opts,'bound')
       Opts.bound = defVal; 
    elseif ~isnumeric(Opts.bound) || ~isscalar(Opts.bound) || Opts.bound < 1 
       error('Wrong value for "Opts.bound"! Has to be an double >= 1!');
    end
end

function Opts = checkMaxIter(Opts,defVal)
% check if the maximum number iterations for fmincon Opts.maxIter takes a 
% correct value

    if ~isfield(Opts,'maxIter')
       Opts.maxIter = defVal; 
    elseif ~isnumeric(Opts.maxIter) || ~isscalar(Opts.maxIter) || ...
           mod(Opts.maxIter,1) ~= 0 || Opts.maxIter < 1 
       error('Wrong value for "Opts.maxIter"! Has to be an integer >= 1!');
    end
end

function Opts = checkRefInput(Opts,defVal)
% check if the flag that specifies if the control input form the reference 
% trajectory should be used for the center Opts.refInput takes a valid
% value

    if ~isfield(Opts,'refInput')
       Opts.refInput = defVal; 
    elseif ~isnumeric(Opts.refInput) || ~isscalar(Opts.refInput) || ...
           (Opts.refInput ~= 0 && Opts.refInput ~= 1) 
       error('Wrong value for "Opts.refInput"! Has to be boolean!');
    end
end

function Opts = checkParallel(Opts,defVal)
% check if the flag Opts.parallel that specifies if parallel computing 
% should be used takes a valid value 

    if ~isfield(Opts,'parallel')
       Opts.parallel = defVal; 
    elseif ~isnumeric(Opts.parallel) || ~isscalar(Opts.parallel) || ...
           (Opts.parallel ~= 0 && Opts.parallel ~= 1) 
       error('Wrong value for "Opts.parallel"! Has to be boolean!');
    end
end

function Opts = checkController(Opts,defVal)
% check if the horizon Opts.extHorizon.decay takes a valid value

    validValues = {'linear','quadratic','exact'};

    if ~isfield(Opts,'controller')
        Opts.controller = defVal;
    elseif ~ischar(Opts.controller) || ...
           ~ismember(Opts.controller,validValues)
        error('Wrong value for "Opts.controller"! Valid values are ''linear'', ''quadratic'', or ''exact''!');
    end
end

function checkOptimizationTime(Opts)
% check if the final time for optimization Opts.tOpt is provided and takes 
% a valid value

    if ~isfield(Opts,'tOpt')
        error('Final time for optimization "Opts.tOpt" is missing!');
    elseif ~isnumeric(Opts.tOpt) || ~isscalar(Opts.tOpt) || Opts.tOpt <= 0
        error('Wrong value for "Opts.tOpt"! Has to be a scalar greater than 0!');
    end
end

function checkTightendInputSet(Opts,Param,nu)
% check if set of tightend input constraints Opts.U_ is provided and takes 
% a valid value

    if ~isfield(Opts,'U_')
        error('Tightend input set "Opts.U_" is missing!');
    elseif ~isa(Opts.U_,'interval')
        error('Wrong value for "Opts.U_"! Has to be object of class "interval"!');
    elseif ~all(size(Opts.U_) == [nu,1])
        error('Wrong dimension for "Opts.U_"! Has to match number of system inputs!');
    elseif ~in(Param.U,Opts.U_)
        error('Wrong value for "Opts.U_"! Has to be a subset of "Param.U"!');
    end
end

function checkTerminalRegion(Opts,Param,nx)
% check if the terminal region Opts.termReg takes a valid value

    if ~isfield(Opts,'termReg')
        error('Terminal region "Opts.termReg" is missing!');
    elseif ~isa(Opts.termReg,'mptPolytope')
        error('Wrong value for "Opts.termReg"! Has to be object of class "mptPolytope"!');
    elseif dim(Opts.termReg) ~= nx
        error('Wrong dimension for "Opts.termReg"! Has to match number of system states!');
    elseif ~in(Opts.termReg,Param.xf)
        error('Wrong value for "Opts.termReg"! Has to contain the final point "Param.xf"!');
    end
end

function checkAllocatedComputationTime(Opts)
% check if the allocated computation time Opts.tComp takes a valid value

    if ~isfield(Opts,'tComp')
        error('Allocated computation time "Opts.tComp" is missing!');
    elseif ~isnumeric(Opts.tComp) || ~isscalar(Opts.tComp) || Opts.tComp <= 0
        error('Wrong value for "Opts.tComp"! Has to be a scalar greater than 0!');
    end
end

function Opts = checkRealTime(Opts,defVal)
% check if the flag specifying if real-time mode is used or not
% Opts.realTime takes a valid value

    if ~isfield(Opts,'realTime')
       Opts.realTime = defVal; 
    elseif ~isscalar(Opts.realTime) || ...
           (Opts.realTime ~= 0 && Opts.realTime ~= 1) 
       error('Wrong value for "Opts.realTime"! Has to be boolean!');
    end
end

function Opts = checkContractionRate(Opts,defVal)
% check if contraction rate Opts.alpha takes a valid value

    if ~isfield(Opts,'alpha')
        Opts.alpha = defVal;
    elseif ~isnumeric(Opts.alpha) || ~isscalar(Opts.alpha) || Opts.alpha <= 0
        error('Wrong value for "Opts.alpha"! Has to be a scalar greater than 0!');
    end
end

function Opts = checkStateWeightingMatrix(Opts,text,nx)
% check if the state weighting matrix Opts.Q takes a valid value

    if ~isfield(Opts,'Q')
       Opts.Q = eye(nx); 
    elseif ~isnumeric(Opts.Q) || ~all(size(Opts.Q) == [nx,nx])
       error(['Wrong value for "Opts.',text,'"! Has to be a square matrix with dimensions matching the system states!']);
    else
       if isdiag(Opts.Q)
          if any(Opts.Q < 0)
              error(['Wrong value for "Opts.',text,'"! Has to be a positive semi-definite matrix!']);
          end
       else
          % check if matrix is positive semi-definite
          [~,D] = eig(Opts.Q);
          if ~isreal(D) || any(D < 0)
              error(['Wrong value for "Opts.',text,'"! Has to be a positive semi-definite matrix!']);
          end
       end
    end
end

function Opts = checkInputWeightingMatrix(Opts,text,nu)
% check if the input weighting matrix Opts.R takes a valid value

    if ~isfield(Opts,'R')
       Opts.R = zeros(nu); 
    elseif ~isnumeric(Opts.R) || ~all(size(Opts.R) == [nu,nu])
       error(['Wrong value for "Opts.',text,'"! Has to be a square matrix with dimensions matching the system states!']);
    else
       if isdiag(Opts.R)
          if any(Opts.R < 0)
              error(['Wrong value for "Opts.',text,'"! Has to be a positive semi-definite matrix!']);
          end
       else
          % check if matrix is positive semi-definite
          [~,D] = eig(Opts.R);
          if ~isreal(D) || any(D < 0)
              error(['Wrong value for "Opts.',text,'"! Has to be a positive semi-definite matrix!']);
          end
       end
    end
end

function Opts = checkStateWeightingMatrixLQR(Opts,nx)
% check if the state weighting matrix for the LQR tracking controller 
% Opts.Qlqr takes a valid value

    if ~isfield(Opts,'Qlqr')
       Opts.Qlqr = eye(nx); 
    elseif ~isnumeric(Opts.Qlqr) || ~all(size(Opts.Qlqr) == [nx,nx])
       error('Wrong value for "Opts.Qlqr"! Has to be a square matrix with dimensions matching the system states!');
    else
       if isdiag(Opts.Qlqr)
          if any(Opts.Qlqr < 0)
              error('Wrong value for "Opts.Qlqr"! Has to be a positive semi-definite matrix!');
          end
       else
          % check if matrix is positive semi-definite
          [~,D] = eig(Opts.Qlqr);
          if ~isreal(D) || any(D < 0)
              error('Wrong value for "Opts.Qlqr"! Has to be a positive semi-definite matrix!');
          end
       end
    end
end

function Opts = checkInputWeightingMatrixLQR(Opts,nu)
% check if the input weighting matrix for the LQR tracking controller 
% Opts.Rlqr takes a valid value

    if ~isfield(Opts,'Rlqr')
       Opts.Rlqr = zeros(nu); 
    elseif ~isnumeric(Opts.Rlqr) || ~all(size(Opts.Rlqr) == [nu,nu])
       error('Wrong value for "Opts.Rlqr"! Has to be a square matrix with dimensions matching the system states!');
    else
       if isdiag(Opts.Rlqr)
          if any(Opts.Rlqr < 0)
              error('Wrong value for "Opts.Rlqr"! Has to be a positive semi-definite matrix!');
          end
       else
          % check if matrix is positive semi-definite
          [~,D] = eig(Opts.Rlqr);
          if ~isreal(D) || any(D < 0)
              error('Wrong value for "Opts.Rlqr"! Has to be a positive semi-definite matrix!');
          end
       end
    end
end

function [Opts,redFields] = checkPolyZono(Opts,defN,defOrderDep,defOrder)
% check if the settings for restructuring polynomial zonotopes
% Opts.polyZono take valid values

    redFields = [];

    if strcmp(Opts.cora.alg,'poly')

       if ~isfield(Opts,'polyZono')
           Opts.polyZono.N = defN;
           Opts.polyZono.orderDep = defOrderDep;
           Opts.polyZono.order = defOrder;
       else
           Opts = checkPolyZonoN(Opts,defN);
           Opts = checkPolyZonoOrderDep(Opts,defOrderDep);
           Opts = checkPolyZonoOrder(Opts,defOrder);
       end
       
       % determine redundant fields
       validFields = {'N','order','orderDep'};
       redFields = getRedundantFields(Opts.polyZono,'polyZono',validFields);
       
    elseif isfield(Opts,'polyZono')
       redFields = 'polyZono';
    end
end

function Opts = checkPolyZonoN(Opts,defVal)
% check if the number of time steps after which the polynomial zonotope is
% restructured Opts.polyZono.N takes a valid value

    if ~isfield(Opts.polyZono,'N')
        Opts.polyZono.N = defVal; 
    elseif ~isnumeric(Opts.polyZono.N) || ~isscalar(Opts.polyZono.N) || ...
           (~isinf(Opts.polyZono.N) && mod(Opts.polyZono.N,1) ~= 0) || ...
           Opts.polyZono.N < 1 
       error('Wrong value for "Opts.polyZono.N"! Has to be an integer >= 1!');
    end
end

function Opts = checkPolyZonoOrderDep(Opts,defVal)
% check if the zonotope order after restructuring for the dependent part
% Opts.polyZono.orderDep takes a valid value

    if ~isfield(Opts.polyZono,'orderDep')
        Opts.polyZono.orderDep = defVal; 
    elseif ~isnumeric(Opts.polyZono.orderDep) || ...
           ~isscalar(Opts.polyZono.orderDep) || Opts.polyZono.orderDep < 1 
       error('Wrong value for "Opts.polyZono.orderDep"! Has to be a double >= 1!');
    end
end

function Opts = checkPolyZonoOrder(Opts,defVal)
% check if the zonotope order after restructuring Opts.polyZono.order takes 
% a valid value

    if ~isfield(Opts.polyZono,'order')
        Opts.polyZono.order = defVal; 
    elseif ~isnumeric(Opts.polyZono.order) || ...
           ~isscalar(Opts.polyZono.order) || Opts.polyZono.order < 1 
       error('Wrong value for "Opts.polyZono.order"! Has to be a double >= 1!');
    end
end

function [Opts,redFields] = checkApprox(Opts,defMethod,defLambda)
% check if the settings for approximating the interpolation control law
% Opts.approx take valid values

    redFields = [];

    if strcmp(Opts.controller,'linear') || strcmp(Opts.controller,'quadratic')
       
       if ~isfield(Opts,'approx')
           Opts.approx.method = defMethod;
           Opts.approx.lambda = defLambda;
       else
           Opts = checkApproxMethod(Opts,defMethod);
           Opts = checkApproxLambda(Opts,defLambda);
       end
       
       % determine redundant fields
       validFields = {'method','lambda'};
       redFields = getRedundantFields(Opts.approx,'approx',validFields);
       
    elseif isfield(Opts,'approx')
        redFields = 'approx';
    end
end

function Opts = checkApproxMethod(Opts,defVal)
% check if method for approximating the interpolation control law
% Opts.approx.method takes a valid value

    validValues = {'scaled','optimized','center'};

    if ~isfield(Opts.approx,'method')
        Opts.approx.method = defVal; 
    elseif ~ischar(Opts.approx.method) || ...
           ~ismember(Opts.approx.method,validValues)
        error('Wrong value for "Opts.approx.method"! Valid values are ''scaled'', ''optimized'', or ''center''!');
    end
end

function Opts = checkApproxLambda(Opts,defVal)
% check if the tradeoff parameter lambda for approximating the
% interpolation control law Opts.approx.lambda takes a valid value

    if ~isfield(Opts.approx,'lambda')
        Opts.approx.lambda = defVal;
    else
        if strcmp(Opts.controller,'linear') && ...
           ~strcmp(Opts.approx.method,'optimized')
            error('For "Opts.controller = ''linear'' the setting "Opts.approx.lambda" is only valid for method "Opts.approx.method = ''optimized''!');
        elseif ~isnumeric(Opts.approx.lambda) || ...
               ~isscalar(Opts.approx.lambda) || ...
               Opts.approx.lambda < 0 || Opts.approx.lambda > 1
            error('Wrong value for "Opts.approx.lambda"! Has to be a scalar value between 0 and 1!');
        end  
    end
end

function [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,alg)
% check if the reference trajectory settings specified in Opts.refTraj take
% valid values

    if ~isfield(Opts,'refTraj')
        Opts.refTraj.Q = eye(nx);
        Opts.refTraj.R = zeros(nu);
        validFields = {'R','Q'};
    elseif isfield(Opts.refTraj,'Q') && isfield(Opts.refTraj,'R')        
        Opts.refTraj = checkStateWeightingMatrix(Opts.refTraj,'refTraj.Q',nx);
        Opts.refTraj = checkInputWeightingMatrix(Opts.refTraj,'refTraj.R',nu);
        validFields = {'R','Q'};
    elseif isfield(Opts.refTraj,'Q') && ~isfield(Opts.refTraj,'R')
        Opts.refTraj = checkStateWeightingMatrix(Opts.refTraj,'refTraj.Q',nx);
        Opts.refTraj.R = zeros(nu);
        validFields = {'R','Q'};
    elseif ~isfield(Opts.refTraj,'Q') && isfield(Opts.refTraj,'R') 
        Opts.refTraj.Q = eye(nx);
        Opts.refTraj = checkInputWeightingMatrix(Opts.refTraj,'refTraj.R',nu);
        validFields = {'R','Q'};
    elseif ~isfield(Opts.refTraj,'x')
        error('States for the reference trajectory "Opts.refTraj.x" are missing!');
    elseif ~isfield(Opts.refTraj,'u')
        error('Inputs for the reference trajectory "Opts.refTraj.u" are missing!');
    else
        checkRefTrajStates(Opts,Param,nx,alg);
        checkRefTrajInputs(Opts,Param,nu,alg);
        validFields = {'x','u'};
    end
    
    % determine redundant fields
    redFields = getRedundantFields(Opts.refTraj,'refTraj',validFields);
end

function checkRefTrajStates(Opts,Param,nx,alg)
% check if the states for the reference trajectory Opts.refTraj.x takes a
% valid value

    % for generator space control (alg == 1) the number of time steps is
    % equal to Opts.N*Opts.Ninter
    if alg == 1
        if ~isnumeric(Opts.refTraj.x)
            error('Wrong value for "Opts.refTraj.x"! Has to be a matrix!');
        elseif size(Opts.refTraj.x,1) ~= nx
            error('Wrong value for "Opts.refTraj.x"! Number of rows has to be equal to number of system states!');
        elseif size(Opts.refTraj.x,2) ~= Opts.N*Opts.Ninter + 1
            error('Wrong value for "Opts.refTraj.x"! Number of columns has to be equal to number of time steps plus one (Opts.N*Opts.Ninter+1)!');
        elseif max(abs(Opts.refTraj.x(:,1)-center(Param.R0))) > 1e-12
            error('Wrong value for "Opts.refTraj.x"! First column has to be identical to the center of the initial set Param.R0!');
        elseif max(abs(Opts.refTraj.x(:,end)-Param.xf)) > 1e-12
            error('Wrong value for "Opts.refTraj.x"! Last column has to be identical to the goal state Param.xf!');
        end
        
    % for all other algorithms (alg == 2) the number of time steps is
    % equal to Opts.N
    else
        if ~isnumeric(Opts.refTraj.x)
            error('Wrong value for "Opts.refTraj.x"! Has to be a matrix!');
        elseif size(Opts.refTraj.x,1) ~= nx
            error('Wrong value for "Opts.refTraj.x"! Number of rows has to be equal to number of system states!');
        elseif size(Opts.refTraj.x,2) ~= Opts.N + 1
            error('Wrong value for "Opts.refTraj.x"! Number of columns has to be equal to number of time steps plus one (Opts.N+1)!');
        elseif max(abs(Opts.refTraj.x(:,1)-center(Param.R0))) > 1e-12
            error('Wrong value for "Opts.refTraj.x"! First column has to be identical to the center of the initial set Param.R0!');
        elseif max(abs(Opts.refTraj.x(:,end)-Param.xf)) > 1e-12
            error('Wrong value for "Opts.refTraj.x"! Last column has to be identical to the goal state Param.xf!');
        end
    end
end

function checkRefTrajInputs(Opts,Param,nu,alg)
% check if the inputs for the reference trajectory Opts.refTraj.u takes a
% valid value

    % for generator space control (alg == 1) the number of time steps is
    % equal to Opts.N*Opts.Ninter
    if alg == 1
        if ~isnumeric(Opts.refTraj.u)
            error('Wrong value for "Opts.refTraj.u"! Has to be a matrix!');
        elseif size(Opts.refTraj.u,1) ~= nu
            error('Wrong value for "Opts.refTraj.u"! Number of rows has to be equal to number of system inputs!');
        elseif size(Opts.refTraj.u,2) ~= Opts.N*Opts.Ninter
            error('Wrong value for "Opts.refTraj.u"! Number of columns has to be equal to number of time steps Opts.N*Opts.Ninter!');
        else
            for i = 1:Opts.N
               if ~in(Param.U,Opts.refTraj.u(:,i))
                   error('Wrong value for "Opts.refTraj.u"! Has to satisfy the input constraints Param.U!');
               end
            end
        end
        
    % for all other algorithms (alg == 2) the number of time steps is
    % equal to Opts.N
    else
        if ~isnumeric(Opts.refTraj.u)
            error('Wrong value for "Opts.refTraj.u"! Has to be a matrix!');
        elseif size(Opts.refTraj.u,1) ~= nu
            error('Wrong value for "Opts.refTraj.u"! Number of rows has to be equal to number of system inputs!');
        elseif size(Opts.refTraj.u,2) ~= Opts.N
            error('Wrong value for "Opts.refTraj.u"! Number of columns has to be equal to number of time steps Opts.N!');
        else
            for i = 1:Opts.N
               if ~in(Param.U,Opts.refTraj.u(:,i))
                   error('Wrong value for "Opts.refTraj.u"! Has to satisfy the input constraints Param.U!');
               end
            end
        end
    end
end

function [Opts,redFields] = checkExtendedHorizon(Opts)
% check if the settings for the extendet optimization horizon are valid

    redFields = {};

    if isfield(Opts,'extHorizon')
       
       validFields = {'active'};
       
       if isfield(Opts.extHorizon,'active')
           if ~isscalar(Opts.extHorizon.active) || ...
              (Opts.extHorizon.active ~= 0 && Opts.extHorizon.active ~= 1)
              error('Wrong value for "Opts.extHorizon.active"! Has to be boolean!');
           elseif Opts.extHorizon.active == 1
              checkHorizon(Opts);
              checkDecay(Opts);
              validFields = {'active','decay','horizon'}; 
           end
       else
           Opts.extHorizon.active = 0;
       end
       
       % determine redundant fields
       redFields = getRedundantFields(Opts.extHorizon,'extHorizon',validFields);
       
    else
       Opts.extHorizon.active = false;
    end
end

function checkHorizon(Opts)
% check if the horizon Opts.extHorizon.horizon takes a valid value

    if ~isfield(Opts.extHorizon,'horizon')
        error('Length of the horizon "Opts.extHorizon.horizon" is missing!'); 
    elseif ischar(Opts.extHorizon.horizon)
        if ~strcmp(Opts.extHorizon.horizon,'all')
            error('Wrong value for "Opts.extHorizon.horizon"! Has to ''all'' or positive integer!');
        end
    elseif ~isnumeric(Opts.extHorizon.horizon) || ...
           ~isscalar(Opts.extHorizon.horizon) || ...
           mod(Opts.extHorizon.horizon,1) ~= 0 || ...
           Opts.extHorizon.horizon < 1
        error('Wrong value for "Opts.extHorizon.horizon"! Has to ''all'' or positive integer!');  
    end
end

function checkDecay(Opts)
% check if the horizon Opts.extHorizon.decay takes a valid value

    validValues = {'fall+End','uniform','fall','fallLinear', ...
                   'fallLinear+End','fallEqDiff','rise','quad', ...
                   'riseLinear','riseEqDiff','end'};

    if ~isfield(Opts.extHorizon,'decay')
        error('Decay function "Opts.extHorizon.decay" is missing!'); 
    elseif ~ischar(Opts.extHorizon.decay) || ...
           ~ismember(Opts.extHorizon.decay,validValues)
        error('Wrong value for "Opts.extHorizon.decay"!');
    end
end

function [Opts,redFields] = checkCoraSettings(Opts,Def)
% check if the settings for the CORA toolbox Opts.cora take valid values

    if ~isfield(Opts,'cora')
        Opts.cora = [];
    end
    
    Opts = checkAlg(Opts,Def);
    Opts = checkZonotopeOrder(Opts,Def);
    Opts = checkTaylorTerms(Opts,Def);
    Opts = checkTensorOrder(Opts,Def);
    Opts = checkIntermediateOrder(Opts,Def);
    Opts = checkErrorOrder(Opts,Def);
    
    % determine redundant fields
    if strcmp(Opts.cora.alg,'lin') && Opts.cora.tensorOrder == 2
        validFields = {'alg','tensorOrder','taylorTerms','zonotopeOrder'}; 
    else
        validFields = {'alg','tensorOrder','taylorTerms', ...
                       'zonotopeOrder','intermediateOrder','errorOrder'}; 
    end
    
    redFields = getRedundantFields(Opts.cora,'cora',validFields);
end

function [Opts,redFields] = checkCoraSettingsLin(Opts,Def)
% check if the settings for the CORA toolbox Opts.cora take valid values
% for linear systems

    if ~isfield(Opts,'cora')
        Opts.cora = [];
    end
    
    Opts = checkLinAlg(Opts,Def);
    Opts = checkZonotopeOrder(Opts,Def);
    Opts = checkTaylorTerms(Opts,Def);
    Opts = checkError(Opts);
    
    % determine redundant fields
    if strcmp(Opts.cora.linAlg,'adap')
        validFields = {'linAlg','error'}; 
    else
        validFields = {'linAlg','taylorTerms','zonotopeOrder'}; 
    end
    
    redFields = getRedundantFields(Opts.cora,'cora',validFields);
end

function Opts = checkAlg(Opts,Def)
% check if reachability algorithm Opts.cora.alg takes a valid value

    validValues = {'lin','poly'};

    if ~isfield(Opts.cora,'alg')
        Opts.cora.alg = Def.alg;     
    elseif ~ischar(Opts.cora.alg) || ~ismember(Opts.cora.alg,validValues)
        error('Wrong value for "Opts.cora.alg"! Has to be ''lin'' or ''poly''!');  
    end
end

function Opts = checkLinAlg(Opts,Def)
% check if reachability algorithm for linear systems Opts.cora.linAlg takes 
% a valid value

    validValues = {'standard','wrapping-free','adap','fromStart'};

    if ~isfield(Opts.cora,'linAlg')
        Opts.cora.linAlg = Def.linAlg;     
    elseif ~ischar(Opts.cora.linAlg) || ~ismember(Opts.cora.linAlg,validValues)
        error('Wrong value for "Opts.cora.linAlg"! Has to be ''standard'',''wrapping-free'',''adap'', or ''fromStart''!');  
    end
end

function Opts = checkZonotopeOrder(Opts,Def)
% check if zonotope order Opts.cora.zonotopeOrder takes a valid value

    if ~isfield(Opts.cora,'zonotopeOrder')
        if ~isfield(Opts.cora,'linAlg') || ~strcmp(Opts.cora.linAlg,'adap')
            Opts.cora.zonotopeOrder = Def.zonotopeOrder;   
        end   
    elseif ~isnumeric(Opts.cora.zonotopeOrder) || ... 
           ~isscalar(Opts.cora.zonotopeOrder) || ...
           mod(Opts.cora.zonotopeOrder,1) ~= 0 || ...
           Opts.cora.zonotopeOrder < 1
        error('Wrong value for "Opts.cora.zonotopeOrder"! Has to be an integer >1!');  
    end
end

function Opts = checkIntermediateOrder(Opts,Def)
% check if intermediate zonotope order Opts.cora.intermediateOrder takes a 
% valid value

    if ~isfield(Opts.cora,'intermediateOrder')
        if ~(strcmp(Opts.cora.alg,'lin') && Opts.cora.tensorOrder == 2)
            Opts.cora.intermediateOrder = Def.intermediateOrder;
        end
    elseif ~isnumeric(Opts.cora.intermediateOrder) || ...
           ~isscalar(Opts.cora.intermediateOrder) || ...
           mod(Opts.cora.intermediateOrder,1) ~= 0 || ...
           Opts.cora.intermediateOrder < 1
        error('Wrong value for "Opts.cora.intermediateOrder"! Has to be an integer >1!');  
    end
end

function Opts = checkErrorOrder(Opts,Def)
% check if error zonotope order Opts.cora.errorOrder takes a valid value

    if ~isfield(Opts.cora,'errorOrder')
        if ~(strcmp(Opts.cora.alg,'lin') && Opts.cora.tensorOrder == 2)
            Opts.cora.errorOrder = Def.errorOrder;
        end
    elseif ~isnumeric(Opts.cora.errorOrder) || ...
           ~isscalar(Opts.cora.errorOrder) || ...
           mod(Opts.cora.errorOrder,1) ~= 0 || ...
           Opts.cora.errorOrder < 1
        error('Wrong value for "Opts.cora.errorOrder"! Has to be an integer >1!');  
    end
end

function Opts = checkTaylorTerms(Opts,Def)
% check if number of taylor terms Opts.cora.taylorTerms takes a valid value

    if ~isfield(Opts.cora,'taylorTerms')
        if ~isfield(Opts.cora,'linAlg') || ~strcmp(Opts.cora.linAlg,'adap')
            Opts.cora.taylorTerms = Def.taylorTerms;   
        end
    elseif ~isnumeric(Opts.cora.taylorTerms) || ... 
           ~isscalar(Opts.cora.taylorTerms) || ...
           mod(Opts.cora.taylorTerms,1) ~= 0 || ...
           Opts.cora.taylorTerms < 1
        error('Wrong value for "Opts.cora.taylorTerms"! Has to be an integer >1!');  
    end
end

function Opts = checkTensorOrder(Opts,Def)
% check if number of tensor order Opts.cora.tensorOrder takes a valid value

    if ~isfield(Opts.cora,'tensorOrder')
        if strcmp(Opts.cora.alg,'poly')
            Opts.cora.tensorOrder = 3;     
        else
            Opts.cora.tensorOrder = Def.tensorOrder; 
        end
    elseif ~isnumeric(Opts.cora.tensorOrder) || ... 
           ~isscalar(Opts.cora.tensorOrder) || ...
           (Opts.cora.tensorOrder ~= 2 && Opts.cora.tensorOrder ~= 3)
        error('Wrong value for "Opts.cora.tensorOrder"! Has to be 2 or 3!');  
    elseif strcmp(Opts.cora.alg,'poly') && Opts.cora.tensorOrder == 2
        error('Wrong value for "Opts.cora.tensorOrder"! For algorithm "Opts.cora.alg = ''poly'' tensor order has to be 3!'); 
    end
end

function Opts = checkError(Opts)
% check if the allowed error Opts.cora.error takes a valid value

    if isfield(Opts.cora,'error')
       if ~isnumeric(Opts.cora.error) || ~isscalar(Opts.cora.error) || ...
           Opts.cora.taylorTerms < eps
            error('Wrong value for "Opts.cora.error"! Has to be a double >0!');  
       end
    end
end

function redFields = getRedundantFields(Opts,name,validFields)
% determine all fields that are redundant and not required

    temp = fields(Opts);
    redFields = {};
    
    for i = 1:length(temp)
       if ~ismember(temp{i},validFields)
          if isempty(name)
             redFields = [redFields,temp(i)]; 
          else
             redFields = [redFields,{[name,'.',temp{i}]}]; 
          end
       end
    end
end