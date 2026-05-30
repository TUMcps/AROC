function res = test_safetyNetControl_factorScaling()
%TEST_SAFETYNETCONTROL_FACTORSCALING - unit test function of factorScaling
%
% Syntax:
%       res = test_computeTermRegNonlinSysLinApproach_checkInvariance
%
% Description:
%       Unit test function for checking the scaling of the initial set so
%       that the constraints (goal set is safely reached) are satisfied.
%       The test scenario is based on [1, Example 1].
%
% Input Arguments:
%       -
%
% Output Arguments:
%       -res:   indicates whether the test has been passed or not
%               (boolean)
%
% See Also:
%       safetyNetControl/scaleInitialSet, safetyNetControl/factorScaling
%
% References:
%       [1] Schuermann et al., "Formal Safety Net Control Using Backward 
%           Reachability Analysis, 2021, IEEE Transactions on Automatic
%           Control
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

% initial set
R0 = zonotope([-8; -2], [1,0,0.1; -2,-0.5,0.1]);

% final reachable set
Rfin = polyZonotope(zeros(2,1), [2,0,1,0; 0,2,1,0.5], [0.1;0],[1,0,3,0; 0,1,1,0; 0,0,0,1], [1;2;3]);

% matrix storing the generator-to-input assignment 
% in this example: the first two generators of R0 are assumed to be due to
% the backward flow of the linearized dynamical system, the last generator
% due to the control inputs
H = [0,0,1];

% goal set
C = [1, 1];
d = 5;
goalSet = polytope(C,d);

% set of admissible states
stateCon = goalSet;

% compute scaling factors
sOpt = factorScaling(R0,Rfin,H,goalSet,stateCon);

% extract the optimal initial and reachable set
Rfin_opt = getSubset(Rfin,Rfin.id,interval(-sOpt,sOpt));
R0_opt = zonotope(R0.c,R0.G*diag(sOpt));

% check satisfaction of the constraints
% -> (zonotope enclosure of) Rfin_opt is contained in the goal set
res_goal = contains(goalSet,zonotope(Rfin_opt));
% -> initial set R0_opt satisfies the state constraints
res_init = contains(stateCon,R0_opt);

res = res_goal && res_init;

end
