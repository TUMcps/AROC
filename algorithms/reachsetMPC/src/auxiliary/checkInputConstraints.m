function res = checkInputConstraints(R,k,u,Opts)
% CHECKINPUTCONSTRAINTS - check if the controller satisfies the input
%                         constraints on all reachable sets
%
% Syntax:
%       [res,maxVal,minVal,Rdiff] = CHECKINPUTCONSTRAINTS(R,Umax,Umin,k,uCenter,scaleFact,options)
%
% Description:
%       This function checks if the given controller satisfies the input
%       constraints on the corresponding reachable set of the controlled 
%       system.
%
% Input Arguments:  
%
%       -R:         Reachable set of the controlled system for the whole
%                   time interval
%       -k:         Feedback matrix for the tracking controller 
%                   (dimension: [nu,nx])
%       -u:         Optimal control inputs for the center trajectory
%                   (dimension: [nu,N])
%       -Opts:      a structure containing following options
%
%           -.nx:           number of states
%           -.nu:           number of inputs
%           -.uMax:         upper bound for the input constraints
%           -.uMin:         lower bound for the input constraints
%
% Output Arguments:
%
%       -res:       Flag that specifies if all constraints are satisfied
%                   (0 or 1)
%
% See Also:
%       reachsetMPC
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

    % initialize parameters
    maxVal = -inf * ones(Opts.nu,1);
    minVal = inf * ones(Opts.nu,1);

    % loop over all intermediate time steps
    for j = 1:length(R)

        % calculate set corresponding to x-x_ref
        temp = zonotope(R{j});
        Z = temp.Z;
        Rdiff = zonotope(Z(1:Opts.nx,:)-Z(Opts.nx+1:end,:));

        % calculate minimal and maximal control input on the set
        Ru = interval(k' * Rdiff);
        maxTemp = supremum(Ru);
        minTemp = infimum(Ru);

        % check if the values are smaller or larger than the previous
        maxVal = max(maxTemp,maxVal);
        minVal = min(minTemp,minVal);
    end

    % add control offset to the minimal and maximal values
    maxVal = maxVal + u;
    minVal = minVal + u;

    % check if the input constraints are fulfilled
    res = 1;

    if any(maxVal > Opts.uMax) || any(minVal < Opts.uMin)
        res = 0;
    end
end