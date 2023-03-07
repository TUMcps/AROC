function [Npoints,fracVert,fracDistVert,distChanges] = defaultSimulateRandom(varargin)
% DEFAULTSIMULATERANDOM - default settings for random simulation
%
% Syntax:
%       [Npoints,fracVert,fracDistVert,distChanges] = DEFAULTSIMULATERANDOM()
%       [Npoints,fracVert,fracDistVert,distChanges] = DEFAULTSIMULATERANDOM(Npoints,fracVert,fracDistVert,distChanges)
%
% Description:
%       Assign default values for the random simulation settings and check
%       if the provided user inputs are correct.
%
% Input Arguments:
%
%       -Npoints:       number of initial points for which trajectories are
%                       simulated
%       -fracVert:      fraction of random initial points corresponding to 
%                       vertices of the initial set (value in [0,1])
%       -fracDistVert:  fraction of random disturbance values corresponding
%                       to vertices of the disturbance set (value in [0,1])
%       -distChanges:   number of changes of the disturbance values during
%                       one time step of the algorithm (integer > 0)
%
% Output Arguments:
% 
%       -Npoints:       number of initial points for which trajectories are
%                       simulated
%       -fracVert:      fraction of random initial points corresponding to 
%                       vertices of the initial set (value in [0,1])
%       -fracDistVert:  fraction of random disturbance values corresponding
%                       to vertices of the disturbance set (value in [0,1])
%       -distChanges:   number of changes of the disturbance values during
%                       one time step of the algorithm (integer > 0)
%
% See Also:
%       simulateRandom
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

    % default values
    Npoints = 20;
    fracVert = 0.5;
    fracDistVert = 0.5;
    distChanges = 10;

    % parse user inputs
    if nargin >= 1 && ~isempty(varargin{1})
       Npoints = varargin{1}; 
    end
    if nargin >= 2 && ~isempty(varargin{2})
       fracVert = varargin{2}; 
    end
    if nargin >= 3 && ~isempty(varargin{3})
       fracDistVert = varargin{3}; 
    end
    if nargin >= 4 && ~isempty(varargin{4})
       distChanges = varargin{4}; 
    end
    
    % check user inputs
    if ~isscalar(Npoints) || Npoints < 1 || mod(Npoints,1) ~= 0
        str = ['Wrong value for input argument "Npoints"!', ...
               ' Has to be a integer > 0!'];
        error(str);
    end
    if ~isscalar(fracVert) || fracVert > 1 || fracVert < 0
        str = ['Wrong value for input argument "fracVert"!', ...
               ' Has to be a double between 0 and 1!'];
        error(str);
    end
    if ~isscalar(fracDistVert) || fracDistVert > 1 || fracDistVert < 0
        str = ['Wrong value for input argument "fracDistVert"!', ...
               ' Has to be a double between 0 and 1!'];
        error(str);
    end
    if ~isscalar(distChanges) || distChanges < 1 || mod(distChanges,1) ~= 0
        str = ['Wrong value for input argument "distChanges"!', ...
               ' Has to be a integer > 0!'];
        error(str);
    end
end