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
% Authors:      Niklas Kochdumper, Felix Gruber, Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % checks for the different control algorithms
    switch name
             
        case 'convexInterpolationControl'
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10,1);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkInteger(Opts,'Ninter',4,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',20,1);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % input weighting matrix Opts.R
            Opts = checkMatrix(Opts,'R',zeros(nu),[nu,nu],true);
            
            % control law Opts.controller
            validValues = {'linear','quadratic','exact'};
            Opts = checkString(Opts,'controller','linear',validValues);
            
            % parallel computing Opts.parallel
            Opts = checkBoolean(Opts,'parallel',true);
            
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
            
            [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
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
            Opts = checkInteger(Opts,'N',10,1);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkInteger(Opts,'Ninter',4,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % input weighting matrix Opts.R
            Opts = checkMatrix(Opts,'R',zeros(nu),[nu,nu],true);
            
            % use inputs from reference trajectory for the center
            Opts = checkBoolean(Opts,'refInput',true);
            
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
            Def.alg = 'lin';
            Def.tensorOrder = 2;
            
            [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','Ninter','reachSteps','Q','R', ...
                           'refInput','refTraj','extHorizon','cora'};
                      
        case 'polynomialControl'

            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10,1);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkInteger(Opts,'Ninter',4,1);

            % polynomial order for the controller
            Opts = checkInteger(Opts,'ctrlOrder',1,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);

            % number of final reachability steps Opts.reachStepsFin
            Opts = checkInteger(Opts,'reachStepsFin',20,1);
            
            % check splits for set of feasible parameters
            Opts = checkInteger(Opts,'splits',0,0);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % check iterative reference trajectory update
            Opts = checkBoolean(Opts,'refUpdate',false);
            
            % use inputs from reference trajectory for the center
            Opts = checkBoolean(Opts,'refInput',true);
            
            % reference trajectory Opts.refTraj
            [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,1);
            
            % check extended horizon
            [Opts,temp] = checkExtendedHorizon(Opts);
            redFields = [redFields,temp];
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 20;         
            Def.zonotopeOrder = 30;     
            Def.intermediateOrder = 20;
            Def.errorOrder = 20;
            Def.alg =  'poly';
            Def.tensorOrder = 3;
            
            [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
            Opts.cora.approxDepOnly = false;
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','Ninter','reachSteps','reachStepsFin', ...
                           'refInput','refTraj','refUpdate','cora','Q',...
                           'ctrlOrder','splits','extHorizon'};
     
        case 'optimizationBasedControl'
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',5,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);
            
            % number of final reachability steps Opts.reachStepsFin
            Opts = checkInteger(Opts,'reachStepsFin',100,1);
            
            % scaling factor between upper and lower bound Opts.bound
            Opts = checkDouble(Opts,'bound',1000,1);
            
            % maximum number of iterations for fmincon Opts.maxIter
            Opts = checkInteger(Opts,'maxIter',15,1);
            
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
                [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
            else
                [Opts,temp] = checkCoraSettingsLin(Opts,Def);
            end
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','reachSteps','reachStepsFin','bound', ...
                           'maxIter','refTraj','cora'};
        
        case 'combinedControl'
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',5,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);
            
            % number of final reachability steps Opts.reachStepsFin
            Opts = checkInteger(Opts,'reachStepsFin',50,1);
            
            % set of tightend input constraints Opts.scale
            Opts = checkScalingFactor(Opts,nu);
            
            % maximum number of iterations for fmincon Opts.maxIter
            Opts = checkInteger(Opts,'maxIter',5,1);
            
            % scaling factor between upper and lower bound Opts.bound
            Opts = checkDouble(Opts,'bound',1000,1);
            
            % use constraint on final states Opts.finStateCon
            Opts = checkBoolean(Opts,'finStateCon',false);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % input weighting matrix Opts.R
            Opts = checkMatrix(Opts,'R',zeros(nu),[nu,nu],true);
            
            % feed-forward state weighting matrix Opts.Qff
            Opts = checkMatrix(Opts,'Qff',eye(nx),[nx,nx],true);
            
            % feed-forward input weighting matrix Opts.Rff
            Opts = checkMatrix(Opts,'Rff',zeros(nu),[nu,nu],true);
            
            % check type of feedforward controller
            validValues = {'genSpace','poly'};
            Opts = checkString(Opts,'feedForward','genSpace',validValues);
            
            % check reference trajectory
            [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,0);
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 50;     
            Def.intermediateOrder = 20;
            Def.errorOrder = 5;
            Def.alg =  'lin';
            Def.linAlg = 'standard';
            Def.tensorOrder = 2;
            
            [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
            redFields = [redFields,temp];           
            
            % valid settings for this algorithm
            validFields = {'N','reachSteps','reachStepsFin', ...
                           'finStateCon','scale','Q','R','Qff','Rff', ...
                           'maxIter','bound','refTraj','cora',...
                           'feedForward'};
                       
        case 'safetyNetControl'
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10,1);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkInteger(Opts,'Ninter',4,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);
            
            % zonotope order for inner approximations Opts.order
            Opts = checkInteger(Opts,'order',3,1);
            
            % refinement iterations for backward reachability Opts.iter
            Opts = checkInteger(Opts,'iter',1,1);
            
            % real-time mode Opts.realTime
            Opts = checkBoolean(Opts,'realTime',false);
            
            % allocated computation time Opts.tComp
            checkAllocCompTimeSafetyNet(Opts,Param);
            
            % comfort controller Opts.controller
            checkComfController(Opts);
            
            % reference trajectory Opts.refTraj
            [Opts,redFields] = checkReferenceTrajectory(Opts,Param,nx,nu,1);
            
            % check reachability settings for CORA toolbox
            if isfield(Opts,'cora')
               if isfield(Opts.cora,'alg')
                   redFields = [redFields,'alg'];
               end
               if isfield(Opts.cora,'tensorOrder')
                   redFields = [redFields,'tensorOrder']; 
               end
            end
            
            Def.taylorTerms = 20;         
            Def.zonotopeOrder = 30;     
            Def.intermediateOrder = 20;
            Def.errorOrder = 5;
            Def.alg = 'poly';
            Def.tensorOrder = 3;
            
            [Opts,temp] = checkCoraSettings(Opts,'cora',Def);
            redFields = [redFields,temp];   

            % valid settings for this algorithm
            validFields = {'N','Ninter','reachSteps','U_','order', ...
                           'iter','realTime','tComp','controller', ...
                           'contrOpts','refTraj','cora'};
                       
        case 'reachsetMPC'
            
            % final time for optimization Opts.tOpt
            checkDouble(Opts,'tOpt',[],0);
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10,1);
            
            % number of intermediate time steps Opts.Ninter
            Opts = checkInteger(Opts,'Ninter',1,1);
            
            % number of reachability steps Opts.reachSteps
            Opts = checkInteger(Opts,'reachSteps',10,1);
            
            % scaling for tightend input/state constraints Opts.scale
            Opts = checkDouble(Opts,'scale',0.9,0,1);
            
            % terminal region Opts.termReg
            Opts = checkTerminalRegion(Opts,Param,nx,false);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % input weighting matrix Opts.R
            Opts = checkMatrix(Opts,'R',zeros(nu),[nu,nu],true);
            
            % state weighting matrix for tracking controller Opts.Qlqr
            Opts = checkMatrix(Opts,'Qlqr',eye(nx),[nx,nx],true);
            
            % input weighting matrix for tracking controller Opts.Rlqr
            Opts = checkMatrix(Opts,'Rlqr',eye(nu),[nu,nu],true);
            
            % real-time mode Opts.realTime
            Opts = checkBoolean(Opts,'realTime',true);
            
            % allocated computation time Opts.tComp
            checkAllocatedComputationTime(Opts);
            
            % contraction rate Opts.alpha
            Opts = checkDouble(Opts,'alpha',0.1,0);
            
            % maximum number of iterations for optimal control Opts.maxIter
            Opts = checkInteger(Opts,'maxIter',10,1);
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 5;     
            Def.intermediateOrder = 5;
            Def.errorOrder = 3;
            Def.alg =  'lin';
            Def.tensorOrder = 2;
            
            [Opts,redFields] = checkCoraSettings(Opts,'cora',Def);          
            
            % valid settings for this algorithm
            validFields = {'tOpt','N','Ninter','reachSteps','Q','R', ...
                           'Qlqr','Rlqr','scale','termReg','realTime', ...
                           'tComp','alpha','maxIter','cora'};
                       
        case 'linSysMPC'
            
            % final time for optimization Opts.tOpt
            checkDouble(Opts,'tOpt',[],0);
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10,1);
            
            % terminal region Opts.termReg
            Opts = checkTerminalRegion(Opts,Param,nx,true);
            
            % state weighting matrix Opts.Q
            Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
            % input weighting matrix Opts.R
            Opts = checkMatrix(Opts,'R',eye(nu),[nu,nu],true);
            
            % feedback matrix Opts.K
            if isfield(Opts,'K')
                checkMatrix(Opts,'K',[],[nu,nx]);
            else
                % state weighting matrix Opts.Qlqr
                Opts = checkMatrix(Opts,'Qlqr',eye(nx),[nx,nx],true);
            
                % input weighting matrix Opts.Rlqr
                Opts = checkMatrix(Opts,'Rlqr',eye(nu),[nu,nu],true);
                
                validFields = {'Rlqr','Qlqr'};
            end
            
            % real-time mode Opts.realTime
            Opts = checkBoolean(Opts,'realTime',true);
            
            % contraction rate Opts.alpha
            Opts = checkDouble(Opts,'alpha',0.1,0);
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 50;     
            Def.linAlg = 'standard';
            
            [Opts,redFields] = checkCoraSettingsLin(Opts,Def); 

            if ~strcmp(Opts.cora.linAlg,'standard')
               redFields = [redFields,'cora.linAlg']; 
            end
            
            % valid settings for this algorithm
            validFields = [validFields, {'tOpt','N','Q','R','Qlqr', ...
                            'Rlqr','termReg','realTime','alpha','cora'}];
                       
        case 'conformantSynthesis'
            
            % set representation used to represent uncertainty
            values = {'interval', 'zonotope'};
            Opts = checkString(Opts,'set','interval',values);
            
            % use measurement error to represent uncertainty
            Opts = checkBoolean(Opts,'measErr',false);
            
            % number of measurements that are grouped together
            Opts = checkInteger(Opts,'group',10,1);

            % trade-off parameter mu
            Opts = checkDouble(Opts,'mu',0.5,0,1);
            
            % valid settings for this algorithm
            validFields = {'set','measErr','group','mu'};
            
            redFields = [];
                       
        case 'subpaving'
            
            % equilibrium point Opts.xEq
            Opts = checkMatrix(Opts,'xEq',zeros(nx,1),[nx,1]);
            
            % check control input for equilibrium point
            Opts = checkMatrix(Opts,'uEq',zeros(nu,1),[nu,1]);
            
            % search domain Opts.Tdomain
            Opts = checkSearchDomain(Opts,nx);
            
            % intial guess Opts.Tinit
            checkInitialDomain(Opts,Param,nx);
            
            % number of box refinements Opts.numRef
            Opts = checkInteger(Opts,'numRef',4,1);
            
            % enlarement factor Opts.enlargeFac
            Opts = checkDouble(Opts,'enlargeFac',1.5,1);
            
            % final time for reachability analysis Opts.tMax
            Opts = checkDouble(Opts,'tMax',[],0);
            
            % check number of reachability steps Opts.reachSteps
            Opts = checkDouble(Opts,'reachSteps',100,0);
            
            % feedback matrix Opts.K
            if isfield(Opts,'K')
                checkMatrix(Opts,'K',[],[nu,nx]);
            else
                % state weighting matrix Opts.Q
                Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
                % input weighting matrix Opts.R
                Opts = checkMatrix(Opts,'R',eye(nu),[nu,nu],true);
                
                validFields = {'R','Q'};
            end
            
            % check reachability settings for CORA toolbox
            Def.taylorTerms = 10;         
            Def.zonotopeOrder = 50;     
            Def.intermediateOrder = 50;
            Def.errorOrder = 5;
            Def.alg =  'lin';
            Def.tensorOrder = 2;
            
            [Opts,redFields] = checkCoraSettings(Opts,'cora',Def);          
            
            % valid settings for this algorithm
            validFields = [validFields, {'Tdomain','Tinit','K' ...
                           'xEq','uEq','numRef','reachSteps','tMax', ...
                           'enlargeFac','cora'}];
            
        case 'zonoLinSys'
            
            validFields = {};
            
            % equilibrium point Opts.xEq
            Opts = checkMatrix(Opts,'xEq',zeros(nx,1),[nx,1]);
            
            % check control input for equilibrium point
            Opts = checkMatrix(Opts,'uEq',zeros(nu,1),[nu,1]);
            
            % search domain Opts.Tdomain
            Opts = checkSearchDomain(Opts,nx,Param);
            
            % time step size for sampled-data controller Opts.timeStep
            checkDouble(Opts,'timeStep',[],0);
            
            % number of time steps Opts.N
            Opts = checkInteger(Opts,'N',10, 1);
            
            % feedback matrix Opts.K
            if isfield(Opts,'K')
                checkMatrix(Opts,'K',[],[nu,nx]);
            else
                % state weighting matrix Opts.Q
                Opts = checkMatrix(Opts,'Q',eye(nx),[nx,nx],true);
            
                % input weighting matrix Opts.R
                Opts = checkMatrix(Opts,'R',eye(nu),[nu,nu],true);
                
                validFields = {'R','Q'};
            end
            
            % convergence criterion Opts.maxDist
            Opts = checkDouble(Opts,'maxDist',1e-2,0);
            
            % method for computing generator matrix Opts.genMethod
            validValues = {'termSet','sampling2D','provided'};
            Opts = checkString(Opts,'genMethod','termSet',validValues);
            
            % generator matrix Opts.G
            if strcmp(Opts.genMethod,'provided')
                checkMatrix(Opts,'G',[],nx);
                validFields = [validFields, {'G'}];
            end
                
            % cost function for the optimization problem Opts.costFun
            validValues = {'geomean','sum','none'};
            Opts = checkString(Opts,'costFun','sum',validValues);        
            
            % check reachability settings for CORA toolbox Opts.cora
            Def.taylorTerms = 4;
            Def.zonotopeOrder = 150;
            Def.linAlg = 'standard';
            
            [Opts,redFields] = checkCoraSettingsLin(Opts, Def);
            
            if ~strcmp(Opts.cora.linAlg,'standard')
               redFields = [redFields,'cora.linAlg']; 
            end
            
            % valid settings for this algorithm
            validFields = [validFields, {'Tdomain','timeStep','N','K', ...
                      'xEq','uEq','maxDist','costFun','genMethod','cora'}];
                       
        otherwise
            error('Wrong value for input argument "name"!');
    end
    
    % warnings if additional fields are provided that are not required
    temp = getRedundantFields(Opts,validFields);
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

function Opts = checkScalingFactor(Opts,nu)
% check if the scaling factor for the tightend inputs constraints
% Opts.scale is provided and takes an valid value

    if ~isfield(Opts,'scale')
        Opts.scale = 0.9*eye(nu);
    elseif ~isnumeric(Opts.scale)
       error(['Wrong value for "Opts.scale! Has to be a scalar or', ...
                ' vector with values between 0 and 1!']);
    else
        if isscalar(Opts.scale)
            Opts.scale = Opts.scale*ones(nu,1);
        end

        if (~all(size(Opts.scale) == [nu,1]) && ...
                ~all(size(Opts.scale) == [1,nu])) || ...
                ~all(Opts.scale <= 1) || ~all(Opts.scale >= 0)
            error(['Wrong value for "Opts.scale! Has to be a scalar or', ...
                   ' vector with values between 0 and 1!']);
        end

        Opts.scale = diag(Opts.scale);
    end
end

function Opts = checkTerminalRegion(Opts,Param,nx,isLin)
% check if the terminal region Opts.termReg takes a valid value

    if ~isfield(Opts,'termReg')
        error('Terminal region "Opts.termReg" is missing!');
    end

    if isa(Opts.termReg,'terminalRegion')
        if ~isLin && isa(Opts.termReg,'termRegSubpaving')
            error(['Wrong value for "Opts.termReg"! Objects of class ', ...
                class(Opts.termReg),' are only allowed for linear systems!']);
        end
        if ~isa(Opts.termReg.set,'mptPolytope')
            Opts.termReg = innerApproxPoly(Opts.termReg.set);
        else
            Opts.termReg = Opts.termReg.set;
        end
    end

    if ~isa(Opts.termReg,'mptPolytope')
        error('Wrong value for "Opts.termReg"! Has to be object of class "mptPolytope" or class "terminalRegion"!');
    elseif dim(Opts.termReg) ~= nx
        error('Wrong dimension for "Opts.termReg"! Has to match number of system states!');
    elseif ~contains(Opts.termReg,Param.xf)
        error('Wrong value for "Opts.termReg"! Has to contain the final point "Param.xf"!');
    end
end

function checkAllocatedComputationTime(Opts)
% check if the allocated computation time Opts.tComp takes a valid value

    if ~isfield(Opts,'tComp')
        error('Allocated computation time "Opts.tComp" is missing!');
    elseif ~isscalar(Opts.tComp) || Opts.tComp <= 0
        error('Wrong value for "Opts.tComp"! Has to be a scalar greater than 0!');
    elseif Opts.tComp > Opts.tOpt/Opts.N
        error('Wrong value for "Opts.tComp"! Has to be smaller than Opts.tOpt/Opts.N!');
    end
end

function checkAllocCompTimeSafetyNet(Opts,Param)
% check if the allocated computation time Opts.tComp takes a valid value

    if ~isfield(Opts,'tComp')
        error('Allocated computation time "Opts.tComp" is missing!');
    elseif ~isscalar(Opts.tComp) || Opts.tComp <= 0
        error('Wrong value for "Opts.tComp"! Has to be a scalar greater than 0!');
    elseif Opts.tComp > Param.tFinal/Opts.N
        error('Wrong value for "Opts.tComp"! Has to be smaller than Param.tFinal/Opts.N!');
    else
        temp = Opts.tComp/(Param.tFinal/(Opts.N*Opts.Ninter));
        if abs(temp - round(temp)) > 1e-12
            error('Wrong value for "Opts.tComp"! Has to be a multiple of Param.tFinal/(Opts.N*Opts.Ninter)!');
        end
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
       error(['Wrong value for "Opts.',text,'"! Has to be a square matrix with dimensions matching the system inputs!']);
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
       redFields = getRedundantFields(Opts.polyZono,validFields,'polyZono');
       
    elseif isfield(Opts,'polyZono')
       redFields = 'polyZono';
    end
end

function Opts = checkPolyZonoN(Opts,defVal)
% check if the number of time steps after which the polynomial zonotope is
% restructured Opts.polyZono.N takes a valid value

    if ~isfield(Opts.polyZono,'N')
        Opts.polyZono.N = defVal; 
    elseif ~isscalar(Opts.polyZono.N) || Opts.polyZono.N < 1 ...
                                        || ~mod(Opts.polyZono.N,1) == 0
       error('Wrong value for "Opts.polyZono.N"! Has to be an integer >= 1!');
    end
end

function Opts = checkPolyZonoOrderDep(Opts,defVal)
% check if the zonotope order after restructuring for the dependent part
% Opts.polyZono.orderDep takes a valid value

    if ~isfield(Opts.polyZono,'orderDep')
        Opts.polyZono.orderDep = defVal; 
    elseif ~isscalar(Opts.polyZono.orderDep) || Opts.polyZono.orderDep < 1 
       error('Wrong value for "Opts.polyZono.orderDep"! Has to be a double >= 1!');
    end
end

function Opts = checkPolyZonoOrder(Opts,defVal)
% check if the zonotope order after restructuring Opts.polyZono.order takes 
% a valid value

    if ~isfield(Opts.polyZono,'order')
        Opts.polyZono.order = defVal; 
    elseif ~isscalar(Opts.polyZono.order) || Opts.polyZono.order < 1 
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
       redFields = getRedundantFields(Opts.approx,validFields,'approx');
       
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
    redFields = getRedundantFields(Opts.refTraj,validFields,'refTraj');
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
               if ~contains(Param.U,Opts.refTraj.u(:,i))
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
               if ~contains(Param.U,Opts.refTraj.u(:,i))
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
       redFields = getRedundantFields(Opts.extHorizon,validFields,'extHorizon');
       
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

function cora = checkReductionTechnique(cora,field,Def)
% check if reductionTechnique takes a valid value

    validValues = {'girard','pca','scott','cluster','combastel', ...
                   'constOpt','methA','methB','methC'};

    if ~isfield(cora,'reductionTechnique')
        cora.reductionTechnique = Def.reductionTechnique;    
    elseif ~ischar(cora.reductionTechnique) || ...
           ~ismember(cora.reductionTechnique,validValues) %~contains(validValues, cora.reductionTechnique)
        error(['Wrong value for "Opts.',field,'.reductionTechnique"!']);  
    end    
end

function [Opts,redFields] = checkCoraSettings(Opts,field,Def)
% check if the settings for the CORA toolbox Opts.cora take valid values

    try
        cora = eval(['Opts.',field]);
    catch
        cora = [];
    end
    if ~isfield(Def,'reductionTechnique')
        Def.reductionTechnique = 'girard';
    end
    cora = checkAlg(cora,field,Def);
    cora = checkZonotopeOrder(cora,field,Def);
    cora = checkTaylorTerms(cora,field,Def);
    cora = checkTensorOrder(cora,field,Def);
    cora = checkIntermediateOrder(cora,field,Def);
    cora = checkErrorOrder(cora,field,Def);
    cora = checkReductionTechnique(cora,field,Def);
    
    % determine redundant fields
    if strcmp(cora.alg,'lin') && cora.tensorOrder == 2
        validFields = {'alg','tensorOrder','taylorTerms','zonotopeOrder',...
                       'reductionTechnique'}; 
    else
        validFields = {'alg','tensorOrder','taylorTerms', ...
                       'zonotopeOrder','intermediateOrder',...
                       'errorOrder','reductionTechnique'}; 
    end
    
    redFields = getRedundantFields(cora,validFields,field);
    eval(['Opts.',field,' = cora;']);
end

function [Opts,redFields] = checkCoraSettingsLin(Opts,Def)
% check if the settings for the CORA toolbox Opts.cora take valid values
% for linear systems

    if ~isfield(Opts,'cora')
        Opts.cora = [];
    end
    
    Opts = checkLinAlg(Opts,Def);
    Opts.cora = checkZonotopeOrder(Opts.cora,'cora',Def);
    Opts.cora = checkTaylorTerms(Opts.cora,'cora',Def);
    Opts.cora = checkError(Opts.cora);
    
    % determine redundant fields
    if strcmp(Opts.cora.linAlg,'adap')
        validFields = {'linAlg','error'}; 
    else
        validFields = {'linAlg','taylorTerms','zonotopeOrder'}; 
    end
    
    redFields = getRedundantFields(Opts.cora,validFields,'cora');    
end

function cora = checkAlg(cora,field,Def)
% check if reachability algorithm Opts.field.alg takes a valid value

    validValues = {'lin','poly','lin-adaptive','lin-poly'};

    if ~isfield(cora,'alg')
        cora.alg = Def.alg;     
    elseif ~ischar(cora.alg) || ~ismember(cora.alg,validValues)
        error(['Wrong value for "Opts.',field,'.alg"! Has to be ''lin'' or ''poly''!']);  
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

function cora = checkZonotopeOrder(cora,field,Def)
% check if zonotope order Opts.cora.zonotopeOrder takes a valid value

    if ~isfield(cora,'zonotopeOrder')
        if ~isfield(cora,'linAlg') || ~strcmp(cora.linAlg,'adap')
            cora.zonotopeOrder = Def.zonotopeOrder;   
        end   
    elseif ~isnumeric(cora.zonotopeOrder) || ... 
           ~isscalar(cora.zonotopeOrder) || ...
           mod(cora.zonotopeOrder,1) ~= 0 || ...
           cora.zonotopeOrder < 1
        error(['Wrong value for "Opts.',field,'.zonotopeOrder"! Has to be an integer >1!']);  
    end
end

function cora = checkIntermediateOrder(cora,field,Def)
% check if intermediate zonotope order Opts.field.intermediateOrder takes a 
% valid value

    if ~isfield(cora,'intermediateOrder')
        if ~(strcmp(cora.alg,'lin') && cora.tensorOrder == 2)
            cora.intermediateOrder = Def.intermediateOrder;
        end
    elseif ~isnumeric(cora.intermediateOrder) || ...
           ~isscalar(cora.intermediateOrder) || ...
           mod(cora.intermediateOrder,1) ~= 0 || ...
           cora.intermediateOrder < 1
        error(['Wrong value for "Opts.',field,'.intermediateOrder"! Has to be an integer >1!']);  
    end
end

function cora = checkErrorOrder(cora,field,Def)
% check if error zonotope order Opts.field.errorOrder takes a valid value

    if ~isfield(cora,'errorOrder')
        if ~(strcmp(cora.alg,'lin') && cora.tensorOrder == 2)
            cora.errorOrder = Def.errorOrder;
        end
    elseif ~isnumeric(cora.errorOrder) || ...
           ~isscalar(cora.errorOrder) || ...
           mod(cora.errorOrder,1) ~= 0 || ...
           cora.errorOrder < 1
        error(['Wrong value for "Opts.',field,'.errorOrder"! Has to be an integer >1!']);  
    end
end

function cora = checkTaylorTerms(cora,field,Def)
% check if number of taylor terms Opts.field.taylorTerms takes a valid value

    if ~isfield(cora,'taylorTerms')
        if ~isfield(cora,'linAlg') || ~strcmp(cora.linAlg,'adap')
            cora.taylorTerms = Def.taylorTerms;   
        end
    elseif ~isnumeric(cora.taylorTerms) || ... 
           ~isscalar(cora.taylorTerms) || ...
           mod(cora.taylorTerms,1) ~= 0 || ...
           cora.taylorTerms < 1
        error(['Wrong value for "Opts.',field,'.taylorTerms"! Has to be an integer >1!']);  
    end
end

function cora = checkTensorOrder(cora,field,Def)
% check if number of tensor order Opts.field.tensorOrder takes a valid value

    if ~isfield(cora,'tensorOrder')
        if strcmp(cora.alg,'poly')
            cora.tensorOrder = 3;     
        else
            cora.tensorOrder = Def.tensorOrder; 
        end
    elseif ~isnumeric(cora.tensorOrder) || ... 
           ~isscalar(cora.tensorOrder) || ...
           cora.tensorOrder <1 
        error(['Wrong value for "Opts.',field,'.tensorOrder"! Has to be >=2!']);  
    elseif strcmp(cora.alg,'lin') && cora.tensorOrder >3
        error(['Wrong value for "Opts.',field,'.tensorOrder"! Has to be 2 or 3 for "Opts.',field,'.alg = ''lin''"!']);
    elseif strcmp(cora.alg,'poly') && cora.tensorOrder == 2  &&...
           ~strcmp(field,'cost.cora')
        error(['Wrong value for "Opts.',field,'.tensorOrder"! For algorithm "Opts.',field,'.alg = ''poly''" tensor order has to be >=3!']); 
    end
end

function cora = checkError(cora,field)
% check if the allowed error Opts.field.error takes a valid value

    if isfield(cora,'error')
       if ~isnumeric(cora.error) || ~isscalar(cora.error) || ...
           cora.taylorTerms < eps
            error(['Wrong value for "Opts.',field,'.error"! Has to be a double >0!']);  
       end
    end
end

function Opts = checkSearchDomain(Opts,nx,varargin)
% check if the search domain Opts.Tdomain exists and takes a valid value

    if ~isfield(Opts,'Tdomain')
        Opts.Tdomain = []; 
        return;
    elseif ~isa(Opts.Tdomain,'interval') || dim(Opts.Tdomain) ~= nx || ...
           ~contains(Opts.Tdomain,Opts.xEq)
        error(['Wrong value for "Opts.Tdomain"! Has to be a ', ...
               num2str(nx),'-dimensional interval containing the ', ...
               'equilibrium point Opts.xEq!']);
    end
    
    if nargin > 2 
        Param = varargin{1};
        if isfield(Param,'X')
            if ~contains(Param.X,Opts.Tdomain)
               error('Search region "Opts.Tdomain" violates the state constraints!'); 
            end
        end
    end
end

function checkInitialDomain(Opts,Param,nx)
% check if the initial guess Opts.Tinit exists and takes a valid value

    if ~isfield(Opts,'Tinit')
        error('Initial guess "Opts.Tinit" is missing!'); 
    elseif ~isa(Opts.Tinit,'interval') || dim(Opts.Tinit) ~= nx || ...
           ~contains(Opts.Tinit,Opts.xEq)
        error('Wrong value for "Opts.Tinit"! Has to be an interval containing the equilibrium point Opts.xEq.');
    elseif isfield(Param,'X') && ~in(Param.X,Opts.Tinit)
        error('Wrong value for "Opts.Tinit"! Has to satisfy the state constraints Param.X.');
    end
end
    
function checkComfController(Opts)
% check if the selected comfort controller Opts.controller exists and takes
% a valid value

    if ~isfield(Opts,'contrOpts')
       error('Comfort controller parameter "Opts.contrOpts" are missing');
    end

    if ~isfield(Opts,'controller')
        error('Comfort controller "Opts.controller" is missing!'); 
    elseif iscell(Opts.controller)
        if ~size(Opts.controller,1) == 1 && ~size(Opts.controller,2) == 1
            error('Wrong value for "Opts.controller"! Has to be a one-dimensional cell-array!');
        elseif ~all(size(Opts.controller) == size(Opts.contrOpts))
            error('Wrong value for "Opts.controller"! Cell-arry size has to match Opts.contrOpts!');
        else
            for i = 1:length(Opts.controller)
                if ~ischar(Opts.controller{i})
                    error('Wrong value for "Opts.controller"! Has to be a string!');
                else
                    name = ['comfContr',Opts.controller{i}];
                    if isempty(which(name))
                        error('Wrong value for "Opts.controller"! Could not find the specified controller!');
                    end
                end
            end
        end
    else
        if ~ischar(Opts.controller)
            error('Wrong value for "Opts.controller"! Has to be a string!');
        else
            name = ['comfContr',Opts.controller];
            if isempty(which(name))
                error('Wrong value for "Opts.controller"! Could not find the specified controller!');
            end
        end
    end
end