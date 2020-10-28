function Opts = settings_optBasedContr_car()
% SETTINGS_OPTBASEDCONTR_CAR - algorithm settings for the car benchmark 
%
% Syntax:
%       Opts = SETTINGS_OPTBASEDCONTR_CAR()
%
% Description:
%       Algorithm settings and parameter for the optimization based control
%       algorithm for the car benchmark
%
% Output Arguments:
%
%       -Opts:                  a structure containing following options
%
%           -.N:                number of time steps
%                               [{5} / positive integer]
%           -.reachSteps:       number of reachability steps during one
%                               time step (optimization)
%                               [{10} / positive integer]
%           -.reachStepsFin:    number of reachability steps during one
%                               time step (final reachable set computation)
%                               [{100} / positive integer]
%           -.maxIter:          maximum number of iterations for
%                               optimization with fmincon 
%                               [{15} / positive integer]
%           -.bound:            scaling factor between upper and lower
%                               bound of the weigthing matrices
%                               [{1000} / positive scalar]
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
%           -.cora.alg:                 reachability algorithm (nonlinear)
%                                       [{'lin'} / 'poly']
%           -.cora.linAlg:              reachability algorithm (linear)
%                                       [{'standard'} / 'fromStart' / 
%                                        'wrapping-free' / 'adap']
%           -.cora.tensorOrder:         taylor order for the abstraction of
%                                       the nonlinear function
%                                       [{2} / 3]
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{50} / positive integer]
%           -.cora.errorOrder:          upper bound for the zonotope order
%                                       before comp. the abstraction error
%                                       [{5} / positive integer]
%           -.cora.intermediateOrder:   upper bound for the zonotope order
%                                       during internal computations
%                                       [{30} / positive integer]
%           -.cora.error:               uppper-bound for Hausdorff distance
%                                       (for cora.linAlg = 'adap' only)
%
% See Also:
%       optimizationBasedControl, car
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

    % number of time steps
    Opts.N = 10;  
    
    % number of reachability analysis time steps
    Opts.reachSteps = 12;
    Opts.reachStepsFin = 100;
    
    % parameters for optimization
    Opts.maxIter = 10;
    Opts.bound = 10000;

    % weighting matrices for the cost function (center trajectory)
    Opts.refTraj.Q = 10*eye(4);             
    Opts.refTraj.R = 1/10*eye(2);
end