function [flagVerified,termReg_verified] = verifyTerminalRegion(obj,dynamics,termReg)
%VERIFYTERMINALREGION - verifies invariance of the terminal region.
%
% Syntax:
%       [flagVerified,termReg_verified] = verifyTerminalRegion(obj,dynamics,termReg)
%
% Description:
%       This function implements the verification procedure described in
%       [1, Sec. III 2]. Compared to the paper, we make the following
%       modification to deal with violations of the time-interval 
%       constraint in [1, Eq. (6b)]: we tighten the polytope, which
%       represents the set of admissible states, in every direction in
%       which the containment condition is violated. The tightening is
%       chosen as the difference of the support function of the reachable
%       set in the violating direction A(j,:) and b(j).
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -dynamics:          instance of class termRegNonlinSysDynamics
%       -termReg:           converged solution
%                           (class: termRegNonlinSysOpt)
%
% Output Arguments:
%       -flagVerified:      true if the verification was succesful, false
%                           otherwise
%                           (logical)
%       -termReg_verified:  verified terminal region
%                           (class: termRegNonlinSysOpt)
%
% See Also:
%       @computeTermRegNonlinSysOpt/checkInvariance
% 
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024.
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

totEnlargeAbsErr = 1.0; 
enlarge = 1.1;
iterVer = 0;

while true
    % compute reachable set and check constraint satisfaction
    [flagInvariance,flagStateConstraints,b_tight] = checkInvariance(obj,dynamics,termReg);
    
    if obj.verbose
        disp(['Invariance contraint satisfied? ',num2str(flagInvariance)])
        disp(['State contraint satisfied? ',num2str(flagStateConstraints)])
    end
    
    flagVerified = flagInvariance && flagStateConstraints;
    if flagVerified || totEnlargeAbsErr > 20
        break
    end

    % ---------- update terminal region ----------------------------------
    iterVer = iterVer + 1;
    disp(['----- iteration ',num2str(iterVer), '------'])

    % tighten constraint set
    obj = obj.tightenStateConstraintSet(min(b_tight,obj.H_X_con.b));

    % bloating factor
    if ~flagInvariance
        totEnlargeAbsErr = enlarge*totEnlargeAbsErr;
    end

    % update terminal region
    % -> initial guess expected to be infeasible during verification loop
    flagInitGuessFeasible = false;
    [obj,termReg,~] = obj.solveOptProblem(dynamics,termReg,flagInitGuessFeasible,totEnlargeAbsErr);
end

termReg_verified = termReg;

end
