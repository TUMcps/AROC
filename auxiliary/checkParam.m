function Param = checkParam(Param,name,nx,nu,nw)
% CHECKPARAM - check if the specified parameters take correct values
%
% Syntax:
%       Param = CHECKPARAM(Param,name,nx,nu,nw)
%
% Description:
%       Checks if all required parameters are specified and that all
%       specified values have the correct format.
%
% Input Arguments:
%
%       -Param:     a structure containing the benchmark parameters
%       -name:      name of the control algorithm, e.g.
%                   'generatorSpaceControl', etc.
%       -nx:        number of system states
%       -nu:        number of inputs
%       -nw:        number of disturbances
%
% Output Arguments:
%  
%       -Param:     a structure containing the benchmark parameters
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
% Authors:      Niklas Kochdumper, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % divide into "Model Predictive Control", "Motion Primitive Control"
    % and "Terminal Region"
    mpcControllers = {'reachsetMPC', 'linSysMPC'};
    motPrimContr = {'generatorSpaceControl','convexInterpolationControl', ...
                    'safetyNetControl','optimizationBasedControl',...
                    'combinedControl', 'polynomialControl'};
    terminalRegion = {'subpaving','zonoLinSys'};

    % different checks depending on the type of control algorithm
    if ismember(name,motPrimContr)

        % initial set
        checkSet('R0', Param, nx, {'interval'});
        
        % input constraints
        checkSet('U', Param, nu, {'interval'});
        
        % set of disturbances
        checkSet('W', Param, nw, {'zonotope', 'interval'});
        
        % goal state
        checkGoalState(Param,nx);
        
        % final time
        checkFinalTime(Param);
        
        % state constraints
        Param = checkStateConstraints(Param,nx); 
        
        % measurement errors
        Param = checkMeasurementErrors(Param,nx);
        
        % define valid fields
        validFields = {'R0','U','W','tFinal','xf','X','V'};
        

    elseif ismember(name,mpcControllers)
        
        % initial set
        checkInitialState(Param,nx);
        
        % input constraints
        checkSet('U', Param, nu, {'interval'});
        
        % set of disturbances
        checkSet('W', Param, nw, {'zonotope', 'interval'}); 
                
        % goal state
        checkGoalState(Param,nx);
            
        % state constraints
        Param = checkStateConstraints(Param,nx);   
        
        % measurement errors
        Param = checkMeasurementErrors(Param,nx);
        
        % define valid fields
        validFields = {'x0','U','W','xf','X','V'};
        
        
    elseif ismember(name,terminalRegion)
        
        % input constraints
        if ismember(name, {'zonoLinSys'})
            checkSet('U', Param, nu, {'mptPolytope', 'interval'})
        else
            checkSet('U', Param, nu, {'interval'});
        end
        
        % set of disturbances
        checkSet('W', Param, nw, {'zonotope', 'interval'}); 
        
        % state constraints
        if isfield(Param,'X')
            checkStateConstraints(Param,nx); 
        end
        
        % measurement errors
        Param = checkMeasurementErrors(Param,nx);
        
        % define valid fields
        validFields = {'U','W','V','X'};
        
    else
        error('Wrong value for input argument "name"!');
    end
    
    % warnings if additional fields are provided that are not required
    temp = fields(Param);
    text = 'The following fields of struct "Param" are redundant: ';
    
    for i = 1:length(temp)
       if ~ismember(temp{i},validFields)
          text = [text,temp{i},', ']; 
       end
    end
    
    if length(text) > 54
       warning(text(1:end-2)); 
    end
end


% Auxiliary Functions -----------------------------------------------------

function checkSet(nameOfSet, Param, n, validSetRepresentations)
% check if the set is provided and takes a valid value

    % check existence + dimensions
    if ~ isfield(Param, nameOfSet)
        error(['The set "Param.', nameOfSet, '" is missing!']);
    elseif ~ all(size(Param.(nameOfSet)) == [n,1])
        error(['Wrong dimension for "Param.', nameOfSet, '"!']);
    end
    
    checkSetRepresentation(nameOfSet, Param, validSetRepresentations);
end

function checkSetRepresentation(nameOfSet, Param, validSetRepresentations)
% check if the set representation is valid

    % init
    setRepresentationIsValid = false;
    
    % iterate over all valid set representations
    for i = 1:length(validSetRepresentations)
        if isa(Param.(nameOfSet), validSetRepresentations{i})
            setRepresentationIsValid = true;
            break;
        end
    end
    
    % throw error if set representation is invalid
    if ~ setRepresentationIsValid
        message = ['Wrong value for "Param.', nameOfSet, '"! Has to be object of class "', validSetRepresentations{1}, '"'];
        for i = 2:length(validSetRepresentations)
            message = [message, ' or "', validSetRepresentations{i}, '"'];
        end
        message = [message, '!'];
        error(message);
    end
end

function Param = checkStateConstraints(Param,nx)
% check if set of state constraints takes a valid value

    if ~isfield(Param,'X')
        Param.X = [];
    elseif dim(Param.X) ~= nx
        error('Wrong dimension for "Param.X"! Has to match number of system states!');
    else
        checkSetRepresentation('X', Param, {'mptPolytope', 'interval'});
    end
end

function Param = checkMeasurementErrors(Param,nx)
% check if set of measurement errors takes a valid value

    if ~isfield(Param,'V')
        Param.V = [];
    elseif dim(Param.V) ~= nx
        error('Wrong dimension for "Param.V"! Has to match number of states!');
    else
        checkSetRepresentation('V', Param, {'zonotope', 'interval'});
    end
end

function checkGoalState(Param,nx)
% check if the goal state is provided and takes a valid value

    if ~isfield(Param,'xf')
        error('Goal state "Param.xf" is missing!');
    elseif ~isnumeric(Param.xf)
        error('Wrong value for "Param.xf"! Has to be a vector!');
    elseif ~all(size(Param.xf) == [nx,1])
        error('Wrong dimension for "Param.xf"! Has to be a column vector!');
    end
end

function checkFinalTime(Param)
% check if final time is provided and takes a valid value

    if ~isfield(Param,'tFinal')
        error('Final time "Param.tFinal" is missing!');
    elseif ~isnumeric(Param.tFinal) || ~isscalar(Param.tFinal) || Param.tFinal <= 0
        error('Wrong value for "Param.tFinal"! Has to be a scalar greater than 0!');
    end
end

function checkInitialState(Param,nx)
% check if initial state is provided and takes a valid value

    if ~isfield(Param,'x0')
        error('Initial state "Param.x0" is missing!');
    elseif ~isnumeric(Param.x0) || ~all(size(Param.x0) == [nx,1])
        error('Wrong value for "Param.x0"! Has to be a vector whos length matches the number of states!');
    end
end