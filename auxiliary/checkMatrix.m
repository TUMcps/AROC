function Opts = checkMatrix(Opts,field,default,dim,varargin)
% CHECKMATRIX - check if a matrix takes as valid value
%
% Syntax:
%       Opts = CHECKMATRIX(Opts,field,default,dim)
%       Opts = CHECKMATRIX(Opts,field,default,dim,posDef)
%
% Description:
%       Checks if the specified matrix is a string and takes a valid 
%       value.
%
% Input Arguments:
%
%       -Opts:      a structure containing the algorithm settings
%       -field:     string specifying the name of the setting that is
%                   checked
%       -default:   default value
%       -dim:       required dimension of the matrix
%       -posDef:    matrix has to be positive definite
%
% Output Arguments:
%
%       -Opts:      adapted structure of algorithm settings
%
% See Also:
%       checkOpts, checkInteger, checkDouble, checkBoolean
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % parse input arguments
    posDef = false;
    
    if nargin > 4
       posDef = varargin{1}; 
    end

    % set to default value if value does not exist
    if ~isfield(Opts,field)
        if ~isempty(default)
            Opts.(field) = default;
            return;
        else
            error(['Setting "Opts.',field,' is missing!']); 
        end
    end

    % check if matrix has valid dimension
    val = Opts.(field);

    if ~isnumeric(val) || ~checkDimension(val,dim)
        str = ['Wrong value for "Opts.',field,'"! Has to a be a ', ...
                        num2str(dim(1)),' x ',num2str(dim(2)), ' matrix!'];
        error(str);
    end
    
    % check if matrix is positive definite
    if posDef
        if ~isPositiveDefinite(val)
            str = ['Wrong value for "Opts.',field,'"! Has to a be a ', ...
                                         'positive semi-definite matrix!'];
            error(str);
        end
    end
end


% Auxiliary Functions -----------------------------------------------------

function res = checkDimension(M,dim)
% check if a matrix has valid dimensionality

    res = true;

    for i = 1:length(dim)
        if size(M,i) ~= dim(i)
            res = false;
            break;
        end
    end
end

function res = isPositiveDefinite(M)
% check if a matrix is positive semi-definite

    res = true;

    if isdiag(M)
        if any(M < 0)
            res = false;
        end
    else
        [~,D] = eig(Opts.Q);
        if ~isreal(D) || any(D < 0)
            res = false;
        end
    end
end