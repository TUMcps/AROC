function Opts = checkString(Opts,field,default,values)
% CHECKSTRING - check if a string setting takes as valid value
%
% Syntax:
%       Opts = CHECKSTRING(Opts,field,default,values)
%
% Description:
%       Checks if the specified setting is a string and takes a valid 
%       value.
%
% Input Arguments:
%
%       -Opts:      a structure containing the algorithm settings
%       -field:     string specifying the name of the setting that is
%                   checked
%       -default:   default value
%       -values:    cell-array defining the valid values
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

    if ~ischar(val) || ~ismember(val,values)
        str = ['Wrong value for "Opts.',field,'"! Valid values are '];
        for i = 1:length(values)
            str = [str, '"', values{i}, '", '];
        end
        str = [str(1:end-2), '!'];
        error(str);
    end
end