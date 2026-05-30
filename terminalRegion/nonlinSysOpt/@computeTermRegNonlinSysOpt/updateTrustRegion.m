function [obj,tr_action] = updateTrustRegion(obj,auxData_iter,absErr_true)
%UPDATETRUSTREGION - updates the trust region according to the strategy
%       described in [1, Appendix II].
%
% Syntax:
%       [obj,tr_action] = updateTrustRegion(obj,auxData_iter,absErr_true)
%
% Description:
%       This function updates the trust region according to the strategy 
%       described in [1, Appendix II].       
%
% Input Arguments:
%       - obj:          updated instance of class
%                       computeTermRegNonlinSysOpt
%       -auxData_iter:  instance of class termRegNonlinSysSolutionAuxData
%       -absErr_true:   set of abstraction errors for the optimal solution
%                       (class: interval)
% 
% Output Arguments:
%       - obj:          updated instance of class
%                       computeTermRegNonlinSysOpt
%       -tr_action:     string describing the modification of the trust  
%                       region
%                       ('enlarge' / 'no action' / 'shrink')
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%-------------------------------------------------------------------

% action in case the size of the trust region does not affect the
% approximation accuracy/the approximation error is not too coarse
tr_action = 'no action';

% size of trust region does not affect the approximation accuracy (optimal
% solution is a subset of the interval enclosure of the previous iterate)
if auxData_iter.trustRegionExploitation == 0
    return
% uniformly enlarge trust region if its fully exploited
elseif auxData_iter.trustRegionExploitation > 0.999
   obj.tr_set = obj.tr_set*obj.tr_grow;
   tr_action = 'enlarge';
% shrink the trust region if the approximation is too coarse
else    
    % compute maximum of relative approximation error
    maxErrRelW = max((auxData_iter.estAbsErr.sup - absErr_true.sup)./ ...
        absErr_true.sup);
    
    % uniformly shrink if the maximum relative approximation error is too 
    % large
    if maxErrRelW > obj.tr_maxRelErr
        obj.tr_set = obj.tr_set*obj.tr_shrink;
        tr_action = 'shrink';
    end
end
