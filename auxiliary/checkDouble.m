function Opts = checkDouble(Opts,field,default,varargin)
% CHECKDOUBLE - check if double is in valid range
%
% Syntax:
%       Opts = CHECKDOUBLE(Opts,field,default)
%       Opts = CHECKDOUBLE(Opts,field,default,lb)
%       Opts = CHECKDOUBLE(Opts,field,default,lb,ub)
%
% Description:
%       Checks if the specified setting is double and takes values in
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
%       checkOpts, checkInteger, checkString
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

    if ~isnumeric(val) || ~isscalar(val) || val < lb || val > ub 
        str = ['Wrong value for "Opts.',field,'"! Has to be a double'];
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