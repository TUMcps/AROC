function Opts = settings_reachsetMPC_artificialSystem_traj1()
% SETTINGS_REACHSETMPC_ARTIFICIALSYSTEM_TRAJ1 - algorithm settings for 
%                                               the artificial system
%                                               benchmark 
%
% Syntax:
%       Opts = SETTINGS_REACHSETMPC_ARTIFICIALSYSTEM_TRAJ1()
%
% Description:
%       Algorithm settings and parameter for the Reachset Model Predictive
%       Control algorithm for the artificial system benchmark.
%
% Output Arguments:
%
%       -Opts:              a structure containing following options
%
%           -.tOpt:         final time for the optimization  
%           -.N:            number of time steps
%                           [{10} / positive integer]
%           -.reachSteps:   number of reachability steps during one time 
%                           step [{10} / positive integer]
%           -.U_:           tightened set of admissible control inputs 
%                           (class: interval)
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope)
%           -.Q:            state weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.R:            input weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.Qlqr:         state weighting matrix for the cost function of
%                           the LQR approach (tracking controller)
%           -.Rlqr:         input weighting matrix for the cost function of
%                           the LQR approach (tracking controller)
%           -.realTime:     flag specifying if real time computation time 
%                           constraints are considered (Opts.realTime = 1) 
%                           or not (Opts.realTime = 0) [{true} / boolean]
%           -.tComp:        time allocated to perform the computations for 
%                           the optimizations (0 < tComp < tOpt/N).
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%           -.maxIter:      maximum number of iterations for the optimal
%                           control problem [{10} / positive integer]
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       [{'lin'} / 'poly']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function [{2}/ 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{5} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{3} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{3} / positive integer]
%
% See Also:
%       reachsetMPC, artificialSystem
%
% References:
%       * *[1] Yu et al. (2013)*, Tube MPC scheme based on robust control 
%              invariant set with application to Lipschitz nonlinear 
%              systems, Systems & Control Letters
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------
    
    % tightend input contraints
    Opts.U_ = interval(-1.5,1.5);
    
    % number of time steps and optimization horizon
    Opts.N = 15;
    Opts.tOpt = 90;
    
    % weighting matrices for the cost function
    Opts.Q = diag([3,3]);             
    Opts.R = 0.1; 
    
    % tracking controller
    Opts.Qlqr = diag([10;10]);
    Opts.Rlqr = 0.2;
    
    % terminal region (see Sec.5 in [1])
    E = ellipsoid(inv([7.9997 -12.2019; -12.2019 27.0777]./10));
    Z = zonotope(E,10,'u:norm:bnd');
    Opts.termReg = mptPolytope(Z);
    
    % algorithm parameter
    Opts.tComp = 2.5;
    Opts.alpha = 0.1;
    Opts.maxIter = 12;
    Opts.reachSteps = 10;

end