function cost = costFunction(obj,y)
%COSTFUNCTION - Cost function of the optimal control problem
%
% Syntax:
%       cost = costFunction(obj,y)
%
% Description:
%       Cost function of the optimal control problem - implements the
%       (possibly time-varying) nominal stage and terminal cost as well as
%       the exact penalty term for the softened input constraints.
%
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -y:         vector of optimization variables
%                   (double array of dimension [obj.idxVars.nVars,1])
%
% Output Arguments:
%       -cost:      value of cost function evaluated at y
%                   (double)
%   
% See Also:
%       optimalControl
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
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% extract variables
idxVars = obj.idxVars;
x = reshape(y(idxVars.x),obj.nx,obj.N+1);
u = reshape(y(idxVars.u),obj.nu,obj.N);
slack_bu = reshape(y(idxVars.slack_bu),idxVars.n_slack_bu/obj.N,obj.N);

% init cost
cost = 0;

% stage cost
for kk = 1:obj.N
    % (time-varying) nominal cost
    cost = cost + full(obj.stageCost{kk}(x(:,kk),u(:,kk)));

    % input bound violation
    cost = cost + obj.softConPenaltyWeight*sum(slack_bu(:,kk));
end

% nominal terminal cost
cost = cost + full(obj.terminalCost(x(:,obj.N+1)));

end
