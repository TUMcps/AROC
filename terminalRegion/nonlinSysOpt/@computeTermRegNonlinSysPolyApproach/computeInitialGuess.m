function initGuess = computeInitialGuess(obj,dynamics,Q,R,weights)
% COMPUTEINITIALGUESS - computes an inital guess for the 
%       polynomialization-based approach for computing terminal regions of 
%       nonlinear systems.
%
% Syntax:
%       initGuess = computeInitialGuess(obj,dynamics,Q,R,weights)
%
% Description:
%       Computes an inital guess for the polynomialization-based approach 
%       for computing terminal regions of nonlinear systems [1]. The 7
%       initial guess for the terminal region itself is obtained by 
%       computing different types of (robust) invariant sets of the 
%       linearized dynamics (around the equilibrium state) and combining 
%       their generators. To obtain an initial guess for the controller, we 
%       use generator space control [2].
%
% Input Arguments:
%       -obj:                   instance of class
%                               computeTermRegNonlinSysPolyApproach
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
%       -initGuess:     initial guess for the terminal region and
%                       controller
%                       (class: termRegNonlinSysPolyApproach)
%           
% See also:
%       @computeTermRegNonlinSysOpt/initialGuessSet_RiccatiEq, 
%       @computeTermRegNonlinSysOpt/initialGuessSet_GSB, 
%       @computeTermRegNonlinSysOpt/initialGuessSet_mRPI
%
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
%       [2] B. Schürmann and M. Althoff, "Guaranteeing Constraints of 
%           Disturbed Nonlinear Systems Using Set-Based Optimal Control in 
%           Generator Space,", IFAC-PapersOnLine, Volume 50, Issue 1, 2017,
%           pp. 11515-11522.
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

% ------- terminal region ------------------------------------------------

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
termReg = reduce(termReg,'girard',obj.termRegOrder);

% represent as polynomianl zonotope
termRegSet = polyZonotope(termReg.c,termReg.G, ...
    [],eye(length(obj.id.termReg)),obj.id.termReg);

% ------- terminal controller  -------------------------------------------

p0 = computeInitialGuessCtrl(obj,dynamics,termRegSet);
ctrl = resolve(obj.pZu_vp,p0,obj.id.p);

% assign output argument
initGuess = termRegNonlinSysPolyApproach(dynamics.dynamicsHandle, ...
    dynamics.sysType,termRegSet,ctrl,p0,dynamics.dt,obj.H_U,obj.W);

end
