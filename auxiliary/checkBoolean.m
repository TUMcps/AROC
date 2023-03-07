function Opts = checkBoolean(Opts,field,default)
% CHECKBOOLEAN - check if setting is a boolean
%
% Syntax:
%       Opts = CHECKBOOLEAN(Opts,field,default)
%
% Description:
%       Checks if the specified setting is boolean.
%
% Input Arguments:
%
%       -Opts:      a structure containing the algorithm settings
%       -field:     string specifying the name of the setting that is
%                   checked
%       -default:   default value
%
% Output Arguments:
%
%       -Opts:      adapted structure of algorithm settings
%
% See Also:
%       checkOpts, checkInteger, checkDouble, checkString
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

    if ~isscalar(val) || ~(islogical(val) || any(val==[0,1]))
        str = ['Wrong value for "Opts.',field,'"! Has to be a boolean!'];
        error(str);
    end
end