function PIset = initialGuessSet_RiccatiEq(obj,dynamics,Q,R)
%INITIALGUESSSET_RICCATIEQ - solves the Riccati equation for the linearized
%       dynamics and inscribes a zonotope into the resulting ellipsoid.
%
% Syntax:
%       PIset = initialGuessSet_RiccatiEq(obj,dynamics,Q,R)
%
% Description:
%       To obtain an initial guess for the optimization-based approaches 
%       for computing terminal regions of nonlinear systems, we solve the 
%       algebraic Riccati equation (ARE) for the linearized dynamics. 
%       Afterwards, a zonotope is inscribed into the positively invariant 
%       ellipsoid whose shape matrix is the solution of the ARE.
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -dynamics:          instance of class termRegNonlinSysDynamics
%       -Q:                 state weighting matrix
%                           (positive-definite matrix of dimension 
%                           [obj.nx,obj.nx])
%       -R:                 control input weighting matrix
%                           (positive-definite matrix of dimension 
%                           [obj.nu,obj.nu])
%
% Output Arguments:
%       -PIset:     approximation of the positively invariant set
%                   (class: termRegNonlinSysLinApproach)
%           
% See also:
%       -
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

% get linearized system
AB_d_nom = dynamics.AB_d.C;
Ad = AB_d_nom(:,1:obj.nx);
Bd = AB_d_nom(:,obj.nx+(1:obj.nu));

% solve the Riccati equation
[minusK,S] = dlqr(Ad,Bd,Q,R);
K = -minusK;

% inner-approximation of the ellipsoid E(inv(S)) by a zonotope
termReg = zonotope(ellipsoid(inv(S)),'inner:norm_bnd',obj.nx*obj.termRegOrder);
termReg = termReg + obj.xEq;

% assign output argument
termRegCtrl = zonotope(obj.uEq, K*termReg.G);
linDynamicsHandle = @(x,u,w) obj.xEq + Ad*(x-obj.xEq) + Bd*(u-obj.uEq);
PIset = termRegNonlinSysLinApproach(linDynamicsHandle,termRegSysType.DT, ...
    termReg,termRegCtrl,dynamics.dt,obj.H_U,zonotope.empty(obj.nx));

end 
