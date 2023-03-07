function Opts = checkInteger(Opts,field,default,varargin)
% CHECKINTEGER - check if integer is in valid range
%
% Syntax:
%       Opts = CHECKINTEGER(Opts,field,default)
%       Opts = CHECKINTEGER(Opts,field,default,lb)
%       Opts = CHECKINTEGER(Opts,field,default,lb,ub)
%
% Description:
%       Checks if the specified setting is an integer and takes values in
%       the valid range.
%
% Input Arguments:
%
%       -Opts:      a structure containing the algorithm settings
%       -field:     string specifying the name of the setting that is
%                   checked
%       -default:   default value
%       -lb:        lower bound of the valid range
%       -ub:        upper bound of the valid range
%
% Output Arguments:
%
%       -Opts:      adapted structure of algorithm settings
%
% See Also:
%       checkOpts, checkDouble, checkString
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
    lb = -inf; ub = inf;

    if nargin > 3
        lb = varargin{1};
    end
    if nargin > 4
        ub = varargin{2};
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

    % check if value is in range
    val = Opts.(field);

    if ~isnumeric(val) || ~isscalar(val) || val < lb ...
                                || val > ub || mod(val,1) ~= 0
        str = ['Wrong value for "Opts.',field,'"! Has to be an integer'];
        if ~isinf(lb)
            if ~isinf(ub)
                str = [str, ' >= ',num2str(lb), ' and <= ',num2str(ub),'!'];
            else
                str = [str, ' >= ',num2str(lb),'!'];
            end
        else
            if ~isinf(ub)
                str = [str, ' <= ',num2str(ub),'!'];
            else
                str = [str,'!'];
            end
        end
        error(str);
    end
end