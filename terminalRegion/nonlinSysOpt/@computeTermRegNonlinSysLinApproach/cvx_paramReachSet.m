function [c_Rparam,G_Rparam] = cvx_paramReachSet(obj,var_s,var_Gctrl,var_RabsEerr,Rfexp,AB_d,RlinW,termRegSet)
%CVX_PARAMREACHSET - computes the parameterized approximation of the
%       time-point reachable set.
%
% Syntax:
%       [c_Rparam,G_Rparam] = cvx_paramReachSet(obj,var_s,var_Gctrl,var_RabsEerr,Rfexp,AB_d,RlinW,termRegSet)
%
% Description:
%       This function computes the center and generator matrix of the
%       zonotope representing the parameterized time-point reachable set as 
%       described in [1, Eq. 19a and Eq. 20].
%
% Input Arguments:
%       -obj:           instance of class
%                       computeTermRegNonlinSysLinApproach
%       -var_s:         generator scaling factors
%                       (positive double vector of dimension
%                       [obj.termRegOrder*obj.nx,1])
%       -var_Gctrl:     generator matrix of zonotopic representation of the
%                       invariance-preserving control law
%                       (double matrix of dimension 
%                       [obj.nu,obj.termRegOrder*obj.nx])
%       -var_RabsEerr:  reachable set due to set of abstraction errors
%                       (struct with the fields:
%                           .c: center of var_Rerr 
%                               (double vector of length nx)
%                           .G: generator matrix of var_Rerr
%                               (double matrix)
%                       )
%       -Rfexp:         reachable set of the linearized system due to the
%                       constant input "dynamics evaluted at expansion 
%                       point"
%                       (class: zonotope)
%       -AB_d:          horizontal concatenation of system matrix and input
%                       matrix of the linearized system dynamics
%                       (class: matZonotope)
%       -RlinW:         reachable set of the linearized dynamics due to the
%                       disturbances
%                       (class: zonotope)
%       -termRegSet:    current candidate for the the terminal region 
%                       (class: zonotope)
%
% Output Argument:
%       -c_Rparam:      center of the parameterized reachable set at the
%                       next time step 
%                       (double array of dimension [nx,1])
%       -G_Rparam:      generator matrix of the parameterized reachable set
%                       at the next time step
%                       (double array)
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
%------------------------------------------------------------------

% center of the reachable set
c_Rparam = obj.xEq + Rfexp.C + var_RabsEerr.c;
if isa(RlinW,'zonotope')
    c_Rparam = c_Rparam + RlinW.c;
end

% generator matrix of the reachable set 
% -> abstraction
G = [termRegSet.G*diag(var_s); var_Gctrl];
G_Rparam = AB_d.C * G;
for ii = 1:AB_d.numgens()
    G_Rparam = [G_Rparam, AB_d.G(:,:,ii)*G];
end
if ~isempty(Rfexp.G)
    G_Rparam = [G_Rparam, Rfexp.G];
end
if isa(RlinW,'zonotope')
    G_Rparam = [G_Rparam, RlinW.G];
end

% -> abstraction error
G_Rparam = [G_Rparam, var_RabsEerr.G];

end
