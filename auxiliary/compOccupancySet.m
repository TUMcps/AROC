function O = compOccupancySet(R,post)
% COMPOCCUPANCYSET - compute the occupancy set from the reachable set
%
% Syntax:
%       O = COMPOCCUPANCYSET(R,post)
%
% Description:
%       This function computes the occupancy set from a previously computed
%       reachable set. The occupancy set stores the set as well as the time
%       of the set.
%
% Input Arguments:
%
%       -R:     object storing the reachable set (class: reachSet)
%       -post:  function handle to the function that computes the occupancy
%               set from the reachable set for the considered benchmark           
%
% Output Arguments:
%
%       -O:     cell-array storing the occupancy set
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

    counter = 1;
    O = {};
    
    % loop over all time-interval reachable sets
    for i = 1:size(R,1)             
       for j = 1:length(R(i).timeInterval.set)
           O{counter}.set = R(i).timeInterval.set{j};
           O{counter}.time = R(i).timeInterval.time{j};
           counter = counter + 1;
       end
    end

    % apply the postprocessing function to compute the occupancy set
    O = post(O);
end