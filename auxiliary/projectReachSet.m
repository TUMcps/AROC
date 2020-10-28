function R = projectReachSet(R,dim)
% PROJECTREACHSET - project the reachSet object to the specified dimension
%
% Syntax:
%       R = PROJECTREACHSET(R,dim)
%
% Description:
%       This function projects an object of class reachSet which stores the
%       time interval as well as the time point reachable sets to the
%       specified dimensions.
%
% Input Arguments:
%
%       -R:     object storing the reachable set (class: reachSet)
%       -dim:   integer vector storing the dimensions onto which the 
%               reachable set is projected          
%
% Output Arguments:
%
%       -R:     object storing the projected reachable set (class: reachSet)
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

    R_ = [];
    
    % loop over all reachable set strands
    for i = 1:size(R,1)
       
        % get time point and time interval solution
        timePoint = R(i).timePoint;
        timeInt = R(i).timeInterval;
        
        % projection
        timeInt.set = cellfun(@(x) project(x,dim),timeInt.set, ...
                             'UniformOutput',false);
        
        timePoint.set = cellfun(@(x) project(x,dim),timePoint.set, ...
                               'UniformOutput',false);
        
        % construct modified reach set object
        Rtemp = reachSet(timePoint,timeInt);
        R_ = add(R_,Rtemp);
    end

    R = R_;
end