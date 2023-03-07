function redFields = getRedundantFields(Opts,validFields,varargin)
% GETREDUNDANTFIELDS - get all redundant fields of a struct
%
% Syntax:
%       redFields = GETREDUNDANTFIELDS(Opts,validFields)
%       redFields = GETREDUNDANTFIELDS(Opts,validFields,name)
%
% Description:
%       Returns a list of all fields of a struct that a redundant.
%
% Input Arguments:
%
%       -Opts:          a structure containing algorithm settings
%       -validFields:   cell-array containing all valid fields
%       -name:          name of the substuct (e.g. 'extHorizon' for fields
%                       of struct Opts.extHorizon)
%
% Output Arguments:
%
%       -redFields:     list of redundant fields in the struct
%
% See Also:
%       checkOpts
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
    name = [];
    if nargin > 2
        name = varargin{1};
    end

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