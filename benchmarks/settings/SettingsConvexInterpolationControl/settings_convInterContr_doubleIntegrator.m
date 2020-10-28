function Opts = settings_convInterContr_doubleIntegrator()
% SETTINGS_CONVINTERCONTR_DOUBLEINTEGRATOR - algorithm settings for the
%                                            double-integrator benchmark 
%
% Syntax:
%       Opts = SETTINGS_CONVINTERCONTR_DOUBLEINTEGRATOR()
%
% Description:
%       Algorithm settings and parameter for the Convex Interpolation
%       Control Algorithm for the double-integrator benchmark
%
% Output Arguments:
%
%       -Opts:              a structure containing following options
%
%           -.controller:   use exact convex interpolation control law or a 
%                           linear or quadratic approximation
%                           [{'linear'} / 'quadratic' / 'exact'] 
%           -.N:            number of time-steps
%                           [{10} / positive integer]
%           -.Ninter:       number of intermediate time steps during one
%                           time step
%                           [{4} / positive integer]
%           -.reachSteps:   number of reachability steps in one time step
%                           [{20} / positive integer]
%           -.Q:            state weighting matrix for the cost function of
%                           the optimal control problem
%                           [{eye(nx)} / positive-definite square matrix]
%           -.R:            input weighting matrix for the cost function of
%                           the optimal control problem
%                           [{zeros(nu)} / positive-definite square matrix]
%           -.parallel:     use parallel computing
%                           [{false} / true]
%
%           -.approx.method:        method used to determine the
%                                   approximated control law
%                                   [{'scaled'} / 'optimized' / 'center']
%           -.approx.lambda:        tradeoff betwen vertex inputs and
%                                   difference from the exact control law
%                                   [{0.5} / value between 0 and 1]
%
%           -.polyZono.N:           number of reference trajectory time
%                                   steps after which the polynomial
%                                   zontope is over-approximated with a
%                                   parallelotope
%                                   [{Opts.N/2} / positive integer]
%           -.polyZono.orderDep:    maximum zonotope order for the 
%                                   dependent part of the polynomial 
%                                   zonotope (for function restructure)
%                                   [{10} / positive integer]
%           -.polyZono.order:       maximum zonotope order for the
%                                   overall polynomial zonotope (for
%                                   function restructure)
%                                   [{20} / positive integer]
%
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in reference trajectory time 
%                                   steps
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
%                           (dimension: [nx,Opts.N + 1])
%           -.refTraj.u     inputs for the user provided reference
%                           trajectory (dimension: [nu,Opts.N])
%
%           -.cora.alg:                 reachability algorithm that is used
%                                       ('lin' or 'poly')
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function (2 or 3)
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{20} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{100} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{50} / positive integer]
%
% See Also:
%       convexInterpolationControl, doubleIntegrator
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
    
    % weighting matrices for the cost function (reference trajectory)
    Opts.refTraj.Q = eye(2)./(0.2^2);             
    Opts.refTraj.R = 1/(9.81^2); 
    
    % weighting matrices for the cost function (vertex trajectories)
    Opts.Q = diag([4,1]);             
    Opts.R = 0; 

end