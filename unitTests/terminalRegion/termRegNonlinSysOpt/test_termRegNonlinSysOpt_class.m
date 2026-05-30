function res = test_termRegNonlinSysOpt_class
%TEST_termRegNonlinSysOpt_CLASS - unit test function of methods
%defined in the class file termRegNonlinSysOpt.
%
% Syntax:
%       res = test_termRegNonlinSysOpt_class
%
% Description:
%       Unit test function for methods defined in the class file 
%       termRegNonlinSysOpt.
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       @termRegNonlinSysOpt
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
%------------------------------------------------------------------ 

% dynamical  system
benchmark = 'test_artificialSystem';
str = ['dynamicsHandle = @(x,u,w)',benchmark,'(x,u,w);'];
eval(str);

% set of admissible states
opts.X = interval(-ones(2,1), ones(2,1));
% set of admissible control inputs
param.U = interval(-ones(2,1), ones(2,1));
% set of uncertain disturbances
param.W = interval(-0.1*ones(2,1), 0.1*ones(2,1));

% algorithm settings
opts.timeStep = 0.1;
opts.xEq = zeros(2,1);
opts.uEq = zeros(2,1);
opts.termRegOrder = 5;
opts.sysType = termRegSysType.SD;

% terminal region
set = zonotope(opts.xEq,[0.5*eye(opts.X.dim),[-0.5;0.5]]);

% instantiate termRegNolinSysOptApproach object
termReg = termRegNonlinSysOpt(dynamicsHandle,opts.sysType,set,opts.timeStep,param.U,param.W);

% --------------- comptue state parameterization -------------------------
nTestCases = 10;
res_stateInSet = false(nTestCases+2,1);

% test case 1: vertex of terminal region
x = [-1;1];
lambda_exp = [-1;1;1];
% ... execute function under test
lambda = termReg.computeStateParameterization(x);
res_stateInSet(1) = all(withinTol(lambda,lambda_exp,1e-8));

% test case 2: state not contained in terminal region
x = [1;1];
lambda = termReg.computeStateParameterization(x);
if max(abs(lambda)) > 1
    res_stateInSet(2) = true;
else
    res_stateInSet(2) = false;   
end

% random testing
X = randPoint(set,nTestCases);
for ii = 3:2+nTestCases
    lambda = termReg.computeStateParameterization(X(:,ii-2));
    if withinTol(X(:,ii-2),set.c+set.G*lambda,1e-6)
        res_stateInSet(ii) = true;
    else
        res_stateInSet(ii) = false;
    end
end

% assign output argument
res = all(res_stateInSet);

end
