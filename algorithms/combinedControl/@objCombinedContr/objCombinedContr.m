classdef objCombinedContr < objController
% OBJCOMBINEDCONTR - construct a controller object for the Combined 
%                    Control Algorithm
%
% Syntax:
%       obj = OBJCOMBINEDCONTR(dyn,Rfin,contrLaw,Param)
%
% Description:
%       Constructor of the controller object for the Combined Control 
%       Algorithm. The object stores all computed data from the 
%       offline-phase of the algorithm. The controller object can be used 
%       to simulate the online-phase of the algorithm or to visualize the 
%       stored data.
%
% Input Arguments: 
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -Rfin:      final reachable set
%
%       -contrLaw:  a structure containing the control law parameter
%
%           -.FeedforwardCtrl:	controller object for the feed-forward
%                               controller (class: objGenSpaceContr or 
%                               objPolyContr)
%           -.K:                cell-array storing the feedback matrices
%           -.linSys:           struct storing the function handles to the
%                               linearized system dynamics
%
%       -Param:     a structure containing the benchmark parameters
%
%           -.R0:           initial set of states (class: interval)
%           -.xf:           goal state
%           -.tFinal:       final time after which the goal state should be
%                           reached
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
% Output Arguments:
%
%       -obj:   resulting object of class objCombinedContr
%
% See Also:
%       combinedControl
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Syntesis developed at the Chair of Robotics, 
% Artificial Intelligence and Embedded Systems, 
% Technische Universitaet Muenchen. 
%
% For updates and further information please visit <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
%
% More Toolbox Info by searching <a href="matlab:docsearch aroc">AROC</a> in the Matlab Documentation
%
%------------------------------------------------------------------
% Authors:      Victor Gassmann, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    FeedforwardCtrl = [];       % feedforward controller (objGenSpaceContr)
    K = [];                     % cell-array storing the feedback matrices
    N = [];                     % number of time steps
    linSys = [];                % linearized system dynamics
    refTraj = [];               % reference trajectory (necessary for simulating xff)
end
   
methods

    function obj = objCombinedContr(dyn,Rfin,contrLaw,Param)
    % class constructor
    
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param);
        
        % store additional properties
        obj.FeedforwardCtrl = contrLaw.FeedforwardCtrl;
        obj.K = contrLaw.K;
        obj.linSys = contrLaw.linSys;
        obj.N = obj.FeedforwardCtrl.Ninter;
        obj.refTraj = contrLaw.refTraj;
    end
end
end