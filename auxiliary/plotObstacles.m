function plotObstacles(varargin)
% PLOTOBSTACLES - plot static and dynamic obstacles
%
% Syntax:
%       PLOTOBSTACLES(statObs)
%       PLOTOBSTACLES(statObs,dynObs,time)
%       PLOTOBSTACLES(statObs,dynObs,time,color,options)
%
% Description:
%       Plot static and dynamic obstacles for the given time frame.
%
% Input Arguments:
%
%       -statObs:   cell-array storing the static obstacles
%       -dynObs:    cell-array storing the dynamic obstacles. Each dynamic
%                   obstacle is represented as a struct with fields .set 
%                   and .time 
%       -time:      time interval for which the trajectory is plotted
%                   (class: interval)
%       -color:     color for the plot specified as a string (e.g. 'r') or
%                   as a vector of RGB-values (e.g. [0 1 0])
%       -options:   additional MATLAB plot options specified as name-value
%                   pairs (e.g. 'LineWidth',2)
%
% See Also:
%       animate
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
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------  

    % parse input arguments
    statObs = {}; dynObs = {}; time = []; 
    color = [255 128 128]/255; NVpairs = {};

    if nargin > 0 && ~isempty(varargin{1})
        statObs = varargin{1};
    end

    if nargin > 1 && ~isempty(varargin{2})
        dynObs = varargin{2};
    end

    if nargin > 2 && ~isempty(varargin{3})
        time = varargin{3};
    end

    if nargin > 3 && ~isempty(varargin{4})
        color = varargin{4};
       if ~ischar(color) && (any(size(color) ~= [3,1]) && ...
          any(size(color) ~= [1,3]))
           error('Wrong value for input argument "color"! Has to be a string or a 3D vector of RGB values!') 
       end
    end

    if nargin > 5
       NVpairs = varargin(5:end); 
    end

    % plot static obstacles
    for i = 1:length(statObs)
        dimension = 1:dim(statObs{i});
        plot(statObs{i},dimension,'FaceColor',color,NVpairs{:});
    end

    % plot dynamic obstacles
    for i = 1:length(dynObs)
        if isempty(time) || isIntersecting(time,dynObs{i}.time)
            dimension = 1:dim(dynObs{i}.set);
            plot(dynObs{i}.set,dimension,'FaceColor',color,NVpairs{:});
        end
    end
end