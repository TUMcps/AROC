function Opts = updateExtHorizon(i,Opts)
% UPDATEEXTHORIZON - update settings for the extended optimization horizon
%
% Syntax:
%       Opts = UPDATEEXTHORIZON(i,Opts)
%
% Description:
%       Update the settings for the extended optimization horizon for the
%       current time step i
%
% Input Arguments:
%
%       -i:         index of the current time step
%       -Opts:      structure containing all options
%
% Output Arguments:
%
%       -Opts:      structure containing the updated options
%
% See Also:
%       decayFunctions, polyomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    if Opts.extHorizon.active
        len_ = Opts.N - i + 1;
    
        if ischar(Opts.extHorizon.horizon)
            if strcmp(Opts.extHorizon.horizon,'all')
               len = len_; 
            else
               error('Wrong value for input argument "Opts.extHorizon.horizon"!');
            end
        else
            len = min(len_,Opts.extHorizon.horizon);
        end
    else
        len = 1;
    end
    Opts.extHorizon.length = len;
    Opts.extHorizon.weights = decayFunctions(len,Opts);
end