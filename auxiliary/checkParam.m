function checkParam(Param,name,nx,nu,nw)
% CHECKPARAM - check if the specified parameters take correct values
%
% Syntax:
%       CHECKPARAM(Param,name,nx,nu,nw)
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

    % divide into "Model Predictive Control" and "Motion Primitive Control"
    mpcControllers = {'reachsetMPC'};
    motPrimContr = {'generatorSpaceControl','convexInterpolationControl', ...
                    'safetyNetControl','optimizationBasedControl'};

    % different checks depending on the type of control algorithm
    if ismember(name,motPrimContr)

        % initial set
        checkInitialSet(Param,nx);
        
        % input constraints
        checkInputSet(Param,nu);
        
        % set of disturbances
        checkDisturbanceSet(Param,nw);
        
        % goal state
        checkGoalState(Param,nx);
        
        % final time
        checkFinalTime(Param);
        
        % state constraints
        if isfield(Param,'X')
            checkStateConstraints(Param,nx); 
        end
        
        % define valid fields
        validFields = {'R0','U','W','tFinal','xf','X'};
        

    elseif ismember(name,mpcControllers)
        
        % initial set
        checkInitialSet(Param,nx);
        
        % input constraints
        checkInputSet(Param,nu);
        
        % set of disturbances
        checkDisturbanceSet(Param,nw);
        
        % goal state
        checkGoalState(Param,nx);
        
        % state constraints
        if isfield(Param,'X')
            error('State constraints are not yet implemented for this algorithm!');
        end
        
        % define valid fields
        validFields = {'R0','U','W','xf'};
        
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

function checkInitialSet(Param,nx)
% check if initial set is provided and takes a valid value

    if ~isfield(Param,'R0')
        error('Initial set "Param.R0" is missing!');
    elseif ~isa(Param.R0,'interval')
        error('Wrong value for "Param.R0"! Has to be object of class "interval"!');
    elseif ~all(size(Param.R0) == [nx,1])
        error('Wrong dimension for "Param.R0"! Has to match number of system states!');
    end
end

function checkInputSet(Param,nu)
% check if input set is provided and takes a valid value

    if ~isfield(Param,'U')
        error('Input set "Param.U" is missing!');
    elseif ~isa(Param.U,'interval')
        error('Wrong value for "Param.U"! Has to be object of class "interval"!');
    elseif ~all(size(Param.U) == [nu,1])
        error('Wrong dimension for "Param.U"! Has to match number of system inputs!');
    end
end

function checkDisturbanceSet(Param,nw)
% check if disturbance set is provided and takes a valid value

    if ~isfield(Param,'W')
        error('Disurbance set "Param.W" is missing!');
    elseif ~isa(Param.W,'interval')
        error('Wrong value for "Param.W"! Has to be object of class "interval"!');
    elseif ~all(size(Param.W) == [nw,1])
        error('Wrong dimension for "Param.W"! Has to match number of disturbances!');
    end
end

function checkStateConstraints(Param,nx)
% check if set of state constraints takes a valid value

    if ~isa(Param.X,'mptPolytope')
        error('Wrong value for "Param.X"! Has to be object of class "mptPolytope"!');
    elseif dim(Param.X) ~= nx
        error('Wrong dimension for "Param.X"! Has to match number of system states!');
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