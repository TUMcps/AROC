function [obj,auxData] = postprocessIterate(obj,s,sCost,slack,exploitTR,estAbsErr,solverTime,setupTime)
%POSTPROCESSITERATE - post-processing of the polynomial programming 
%       solution.
%
% Syntax:
%       [obj,auxData] = postprocessIterate(obj,s,sCost,slack,tr_exploit,estAbsErr,solverTime,setupTime)
%
% Description:
%       In addition to the post-processing implemented in the superclass
%       method, this function additionally updates the controller template
%       for the next iteration.
%
% Input Arguments:
%       -obj:           instance of class 
%                       computeTermRegNonlinSysPolyApproach
%       -s:             generator scaling factors for the terminal region
%                       (double vector)
%       -sCost:         generator scaling factors for obj.costSet 
%                       (double vector)
%       -slack:         vector collecting all slack variables
%                       (double vector)
%       -exploitTR:     scalar indicating the exploitation of the trust
%                       region
%                       (scalar, 0 <= tr_exploit <= 1)
%       -estAbsErr:     estimate of the set of abstraction errors
%                       (class: interval)
%       -solverTime:    time for solving the optimization problem
%                       (positive scalar)
%       -setupTime:     time for setting up the optimization problem
%                       (positive scalar)
%
% Output Arguments:
%       -obj:       updated instance of class
%                   computeTermRegNonlinSyPolyApproach
%       -auxData:   additional solution data (e.g. flag indicating 
%                   feasibility)
%                   (class: termRegNonlinSysSolutionAuxData)
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller 
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
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
%------------------------------------------------------------------

% call superclass function
[obj,auxData] = postprocessIterate@computeTermRegNonlinSysOpt(obj, ...
    sCost,slack,exploitTR,estAbsErr,solverTime,setupTime);

% update controller template
obj = obj.updateCtrlTemplate(s);

end
