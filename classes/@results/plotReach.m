function han = plotReach(obj,varargin)
% PLOTREACH - plot the computed reachable set
%
% Syntax:
%       han = PLOTREACH(obj)
%       han = PLOTREACH(obj,dim)
%       han = PLOTREACH(obj,dim,color)
%       han = PLOTREACH(obj,dim,color,options)
%
% Description:
%       Plot the reachable set stored in the results object
%
% Input Arguments:
%
%       -obj:       object of class results
%       -dim:       2D-vector containing the dimensions that are plotted
%       -color:     color for the plot specified as a string (e.g. 'r') or
%                   as a vector of RGB-values (e.g. [0 1 0])
%       -options:   additional MATLAB plot options specified as name-value
%                   pairs (e.g. 'LineWidth',2)
%
% Output Arguments:
%       -han:       handle pointing to the plotted set
%
% See Also:
%       results, plotReachTimePoint, plotSimulation
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
    dim = [1,2];
    color = [.6 .6 .6];
    edgeColor = 'none';
    NVpairs = [];

    % parse input arguments
    if nargin > 1 && ~isempty(varargin{1})
       dim = varargin{1};
       if (any(size(dim) ~= [2,1]) && any(size(dim) ~= [1,2])) || ...
           any(mod(dim,1) ~= 0)
          error('Wrong value for input argument "dim"! Has to be a 2D vector of integers!') 
       end
    end
    
    if nargin > 2 && ~isempty(varargin{2})
       color = varargin{2};
       if ~ischar(color) && (any(size(color) ~= [3,1]) && ...
          any(size(color) ~= [1,3]))
           error('Wrong value for input argument "color"! Has to be a string or a 3D vector of RGB values!') 
       end
    end
    
    if nargin > 3
       NVpairs = varargin(3:end); 
       color = getNameValuePair(NVpairs,'FaceColor',color);
       edgeColor = getNameValuePair(NVpairs,'EdgeColor',edgeColor);
    end

    % plot the reachable set
    if isempty(NVpairs)
        han = plot(obj.reachSet,dim,'FaceColor',color,'EdgeColor',edgeColor);
    else
        han = plot(obj.reachSet,dim,'FaceColor',color,'EdgeColor',edgeColor,NVpairs{:});
    end
end