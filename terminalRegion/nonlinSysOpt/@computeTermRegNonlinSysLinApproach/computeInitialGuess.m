function initGuess = computeInitialGuess(obj,dynamics,Q,R,weights)
% COMPUTEINITIALGUESS - computes an inital guess for the 
%       linearization-based approach for computing terminal regions of 
%       nonlinear systems.
%
% Syntax:
%       initGuess = computeInitialGuess(obj,dynamics,Q,R,weights)
%
% Description:
%       Computes an inital guess for the linearization-based approach for 
%       computing terminal regions of nonlinear systems [1]. The initial
%       guess is obtained by computing different types of (robust)
%       invariant sets of the linearized dynamics (around the equilibrium
%       state) and combining their generators.
%
% Input Arguments:
%       -obj:                   instance of class
%                               computeTermRegNonlinSysLinApproach
%       -dynamics:              instance of class termRegNonlinSysDynamics                        
%       -Q:                     state weighting matrix
%                               (positive-definite matrix of dimension 
%                               [obj.nx,obj.nx])
%       -R:                     control input weighting matrix
%                               (positive-definite matrix of dimension 
%                               [obj.nu,obj.nu])
%       -weights:               struct storing the weights for computing
%                               the initial guess
%       
% Output Arguments:
%       -initGuess:     initial guess for the terminal region
%                       (class: termRegNonlinSysLinApproach)
%           
% See also:
%       @computeTermRegNonlinSysOpt/initialGuessSet_RiccatiEq, 
%       @computeTermRegNonlinSysOpt/initialGuessSet_GSB, 
%       @computeTermRegNonlinSysOpt/initialGuessSet_mRPI
%
% References:
%       [1] L. Schäfer et al., "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770. 
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

% zonotopic enclosure of ellipsoid induced by the solution of the
% discrete-time Riccati equation
PI = initialGuessSet_RiccatiEq(obj,dynamics,Q,R);

% outer-approximation of the minimum RCI set 
minRCI = initialGuessSet_GSB(obj,dynamics);

% outer-approximation of the minimal RPI set of the linearized dynamics
minRPI = initialGuessSet_mRPI(obj,dynamics,Q,R);

% combine initial guesses
termReg = zonotope(obj.xEq,[weights.RiccatiEq*PI.set.G, ...
    weights.minRCI*minRCI.set.G, weights.minRPI*minRPI.set.G]);

% add backward reachability generators
% ... linearized system around the equilibrium state
AB_d_nom = dynamics.AB_d.C;
Ad = AB_d_nom(:,1:obj.nx);
Bd = AB_d_nom(:,obj.nx+(1:obj.nu));
% ... scaled input set
width = weights.bwReachInputs*min([interval(obj.Z_U).sup-obj.uEq, obj.uEq-interval(obj.Z_U).inf],[],2);
tmp_U = zonotope(interval(obj.uEq-width, obj.uEq+width));
% ... backward reachability analysis 
Adinv = inv(Ad);
termReg = zonotope(obj.xEq, Adinv*termReg.G) + Adinv*(-1)*Bd*(tmp_U + zonotope(-obj.uEq));
% ... order reduction
termReg = reduce(termReg,'pca',obj.termRegOrder);

% assign output argument
initGuess = termRegNonlinSysLinApproach(dynamics.dynamicsHandle,dynamics.sysType, ...
    termReg,zonotope([]),dynamics.dt,obj.H_U,obj.W);

end
