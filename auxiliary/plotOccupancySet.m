function han = plotOccupancySet(O,dim,color)
% PLOTOCCUPANCYSET - plot the occupancy set
%
% Syntax:
%       han = PLOTOCCUPANCYSET(O,dim,color)
%
% Description:
%       This function plots the occopancy set for one motion primitive.
%
% Input Arguments:
%
%       -O:     cell-array storing the occupancy set
%       -dim:   integer vector of length two specifying the dimensions that
%               should be plotted
%       -color: color of the plot (string of vector with RGB values)      
%
% Output Arguments:
%
%       -han:   handle to the plotted object
%
% See Also:
%       maneuverAutomaton
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

    hold on
    for i = 1:length(O)
        han = plot(O{i}.set,dim,color,'Filled',true,'EdgeColor','none');
    end
end