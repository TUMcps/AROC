function res = test_termRegNonlinSysLinApproach_class
%TEST_TERMREGNONLINSYSLINAPPROACH_CLASS - unit test function of methods
%defined in the class file termRegNonlinSysLinApproach.
%
% Syntax:
%       res = test_termRegNonlinSysLinApproach_class
%
% Description:
%       Unit test function for methods defined in the class file 
%       termRegNonlinSysLinApproach.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @termRegNonlinSysLinApproach
%
% References:
%       -
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

% dynamical  system
benchmark = 'test_artificialSystem';
str = ['dynamicsHandle = @(x,u,w)',benchmark,'(x,u,w);'];
eval(str);

% set of admissible states
param.X = interval(-ones(2,1), ones(2,1));
% set of admissible control inputs
param.U = interval(-ones(2,1), ones(2,1));
% set of uncertain disturbances
param.W = interval(-0.1*ones(2,1), 0.1*ones(2,1));

% algorithm settings
opts.dt = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = zeros(2,1);
opts.termRegOrder = 5;

% terminal region
set = zonotope(opts.xEq,[0.5*eye(param.X.dim),[-0.5;0.5]]);
% controller set (for testing purposes, we do not require the control law
% to be invariance-preserving)
K = -eye(param.X.dim);
ctrlSet = zonotope(opts.uEq,K*set.G);
% sampled-data system (dummy argument)
sysType = termRegSysType.SD;

% instantiate termRegNolinSysOptApproach object
termReg = termRegNonlinSysLinApproach(dynamicsHandle,sysType,set,ctrlSet, ...
    opts.dt,param.U,param.W);

% --------------- compute control input ----------------------------------
nTestCases = 10;
res_ctrl = false(nTestCases,1);

% random testing
X = randPoint(set,nTestCases);
for ii = 1:nTestCases
    u = termReg.computeControlInput(X(:,ii));
    u_exp = K*(X(:,ii)-opts.xEq);
    if withinTol(u,u_exp,1e-8)
        res_ctrl(ii) = true;
    else
        res_ctrl(ii) = false;
    end
end

% assign output argument
res = all(res_ctrl);

end
