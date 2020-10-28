function val = getNameValuePair(NVpairs,name,varargin)
% GETNAMEVALUEPAIR - returns the value of a name-value-pair for plotting
%
% Syntax:
%       val = GETNAMEVALUEPAIR(NVpairs,name)
%       val = GETNAMEVALUEPAIR(NVpairs,name,defVal)
%
% Description:
%       This function takes a list of name-value-pairs specified by the
%       user as additional options for a plot function (e.g. 'LineWidth',2)
%       and returns the value of the selected option.
%
% Input Arguments:
%
%       -NVpairs:   cell-array storing the name-value-pairs passed to the
%                   plot function
%       -name:      string containing the name for the name-value-pair
%       -defVal:    default value if the name value pair is not specified
%                   by the user
%
% Output Arguments:
%       -val:       specified value for the name value pair
%
% See Also:
%       results, plotReach, plotReachTimePoint, plotSimulation
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

    % parse input arguments
    if nargin < 3
        val = [];
    else
        val = varargin{1};
    end 
        
    % check every second entry
    for i=1:2:length(NVpairs)-1
    
        % has to be a char
        if ischar(NVpairs{i})
        
            % name found
            if strcmp(NVpairs{i},name)
                val = NVpairs{i+1};
            end
        end
    end
end