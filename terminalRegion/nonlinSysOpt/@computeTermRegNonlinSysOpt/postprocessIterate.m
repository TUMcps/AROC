function [obj,auxData] = postprocessIterate(obj,sCost,slack,tr_exploit,estAbsErr,solverTime,setupTime)
%POSTPROCESSITERATE - post-processing of the convex/polynomial programming 
%       solution.
%
% Syntax:
%       [sol,costCoeffNext,costSetNext] = postprocessIterate(obj,sCost,slack,tr_exploit,estAbsErr,solverTime,setupTime)
%
% Description:
%       The post-processing of the optimal solution consists of the 
%       following steps:
%           - checking feasibility
%           - computation of the volume of the (under-approxmiation) of the
%           candidate terminal region (required for checking convergence)
%           - updating the coefficients of the cost function for the next
%           iteration
%
% Input Arguments:
%       -obj:           instance of class 
%                       computeTermRegNonlinSysOpt
%       -sCost:         generator scaling factors for obj.costSet 
%                       (double vector)
%       -slack:         vector collecting all slack variables
%                       (double vector)
%       -tr_exploit:    scalar indicating the exploitation of the trust
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
%                   computeTermRegNonlinSysOpt
%       -auxData:   additional solution data (e.g. flag indicating 
%                   feasibility)
%                   (class: termRegNonlinSysSolutionAuxData)
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
%       [2] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
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

% check feasibility
flagIsFeasible = max(slack) <= 0;

% update cost set
obj = obj.updateCostSet(sCost);
% ... evaluate monominals of volume function addends
monomials = prod(sCost(obj.costExponents),2);
% ... update coefficients
obj.costCoefficients = obj.costCoefficients.*monomials;

% instantiate auxiliary data object
auxData = termRegNonlinSysSolutionAuxData( ...
    flagIsFeasible,obj.costSet,tr_exploit,estAbsErr,solverTime,setupTime);

end
