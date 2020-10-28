function Opts = parseOpts(Opts,Def)
% PARSEOPTS - creates an struct with algorithm setting after parsing
%
% Syntax:
%       Opts = PARSEOPTS(Opts, Def)
%
% Description:
%       Creates a struct containing the resulting algorihtm options from
%       the user input and the default values.
%
% Input Arguments:
%
%       -Opts:          Structure containing the algorithm options
%                       specified by the user
%       -Def:           Structure containing the default algorithm options       
%
% Output Arguments:
%
%       -Opts:          Structure containing the resulting algorithm
%                       options
%
% See Also:
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

    fnames = fieldnames(Def); %target field names

    for k = 1:length(fnames)
        if isstruct(Def.(fnames{k})) %nested structure
            % call parseOpts with the nested structure!
              % make sure the nested structure is defined 
                if ~isfield(Opts,fnames{k})
                    Opts.(fnames{k}) = struct;
                end
              % call the function again
                Opts.(fnames{k}) = parseOpts(Opts.(fnames{k}),Def.(fnames{k}));
        else
            if ~isfield(Opts,(fnames{k})) || isempty(Opts.(fnames{k}))
                Opts.(fnames{k}) = Def.(fnames{k});
            end
        end
    end