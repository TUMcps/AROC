function Opts = settings_genSpaceContr_cart()
% SETTINGS_GENSPACECONTR_CART - algorithm settings for the cart benchmark 
%
% Syntax:
%       Opts = SETTINGS_GENSPACECONTR_CART()
%
% Description:
%       Algorithm settings and parameter for the controller synthesis 
%       algorithm that is based on optimal control in generator space for 
%       the cart benchmark.
%
% Output Arguments:
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.N:            number of time steps
%                           [{10} / positive integer]
%           -.Ninter:       number of intermediate time steps during one
%                           time step
%                           [{4} / positive integer]
%           -.reachSteps:   number of reachability steps in one time step
%                           [{10} / positive integer]
%           -.Q:            state weighting matrix for the cost function of
%                           the optimal control problem
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the cost function of
%                           the optimal control problem
%                           [{zeros(nu)} / positive-definite square matrix]
%           -.refInput:     use the input from the reference trajectory
%                           as input for the center instead of optimizing
%                           [{false} / boolean]
%
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in center trajectory time steps
%                                   [{'all'} / positive integer]
%           -.extHorizon.decay:     decay function for the objective
%                                   function of the optimization problem
%                                   with extended optimization horizon
%                                   [{'fall+End'} / 'uniform' / 'fall' / 
%                                    'fallLinear' / 'fallLinear+End' / 
%                                    'fallEqDiff' / 'fallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
%           
%           -.refTraj.Q:    state weighting matrix for the cost function of
%                           optimal control problem (dimension:[nx,nx])
%           -.refTraj.R:    input weighting matrix for the cost function of
%                           optimal control problem (dimension:[nu,nu])
%           -.refTraj.x:    user provided reference trajectory
%                           (dimension: [nx,Opts.N*Opts.Ninter + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Opts.N*Opts.Ninter])
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       [{'lin'} / 'poly']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function [{2}/ 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{20} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{30} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{20} / positive integer]
%
% See Also:
%       generatorSpaceControl, car
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
    
    % number of center trajectory and intermediate time steps
    Opts.N = 10;
    Opts.Ninter = 4;
    
    % weighting matrices for the cost function (alpha optimization)
    Opts.Q = diag([2,1]);             
    Opts.R = 0; 
    
    % weighting matrices for the cost function (reference trajectory)
    Opts.refTraj.Q = eye(2)./(0.2^2);
    Opts.refTraj.R = 1/(14^2);
    
end