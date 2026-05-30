function [flagInvariance,flagStateConstraints,b_tight] = checkInvariance(obj,dynamics,termReg)
%CHECKINVARIANCE - computes the reachable set of the dynamical system and
%       checks whether the the terminal region is a robust control invariant set.
%
% Syntax:
%       [flagInvariance,flagStateConstraints,b_tight] = checkInvariance(obj,dynamics,termReg)
%
% Description:
%       This function is part of the verification procedure described in
%       [1, Sec. III 2]: first over-approximations of the reachable sets
%       are computed and, second, satisfaction of the constraints in [1,
%       Eq. (6a), (6b)] / [2, Eq. (6b),(6c)] is checked. If the state 
%       constraint in [1, Eq. (6b)] / [2, Eq. (6c)] is violated, the 
%       difference of the support function of the reachable set in the 
%       violating direction A(j,:) and b(j) are computed (the set of 
%       admissible states is assumed to be represented as a polytope 
%       {x: A*x <= b}). This difference is used to tighten the set 
%       of admissible states in the next iteration of the verification 
%       procedure ({A*x <= b - b_tight}).
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -dynamics:          instance of class termRegNonlinSysDynamics
%       -termReg:           converged solution
%                           (class: termRegNonlinSysOpt)
%
% Output Arguments:
%       -flagInvariance:        indicates satisfaction of the invariance
%                               constraint in [1, Eq. 6a]
%                               (class: logical)
%       -flagStateConstraints:  indicates satisfaction of the state 
%                               constraint in [1, Eq. 6b]
%                               (class: logical)
%       -b_tight:               tightening vector for the state constraint
%                               in [1, Eq. 6b]
%                               (non-negative double vector)                           
%
% See Also:
%       @computeTermRegNonlinSysOpt/verifyTerminalRegion
% 
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024.
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

% ---------- reachability analysis ---------------------------------------

% parameters for reachability analysis
reachParams = dynamics.reachParams;
if isa(termReg,'termRegNonlinSysLinApproach')
    reachParams.R0 = zonotope([termReg.set.c;termReg.ctrlSet.c], ...
        [termReg.set.G;termReg.ctrlSet.G]);
elseif isa(termReg,'termRegNonlinSysPolyApproach')
    reachParams.R0 = stack(termReg.set,termReg.ctrlSet);
end

% reachability analysis - perturbed system
R = reach(dynamics.sys,reachParams,dynamics.reachOpts);
RnextW = project(R.timePoint.set{end},1:obj.nx);

% ---------- check constraint satisfaction -------------------------------

% check invariance of the terminal region [1, Eq. (6a)] / [2, Eq. (6b)]
if ~contains(zonotope(termReg.set),zonotope(RnextW),'approx:st',1e-6)
    flagInvariance = false;
else
    flagInvariance = true;
end

% check cont.-time satisfaction of state constraints [1, Eq. 6b], [2, Eq. (6c)]
flagStateConstraints = true;
if ~isempty(R.timeInterval)
    % sampled-data systems
    b_tight = zeros(size(obj.H_X.b));
    for ii=1:length(R.timeInterval.set)
        tmp_Rti = zonotope(project(R.timeInterval.set{ii},1:obj.nx));
        b_error = obj.H_X.A*tmp_Rti.c + sum(abs(obj.H_X.A*tmp_Rti.G),2) - obj.H_X.b;
        b_error = max(b_error,0);
        if max(b_error) > 0
            flagStateConstraints = false;
            b_tight = max(b_tight,1.1*b_error);
        end
    end
else
    % discrete-time systems (the terminal region has to be contained in the 
    % set of admissible states)
    b_tight = zeros(size(obj.H_X.b));
    b_error = obj.H_X.A*termReg.set.c + sum(abs(obj.H_X.A*termReg.set.G),2) - obj.H_X.b;
    b_error = max(b_error,0);
    if max(b_error) > 0
        flagStateConstraints = false;
        b_tight = max(b_tight,b_error);
    end       
end

end
