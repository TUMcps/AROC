classdef objConvInterContr < objController
% OBJCONVINTERCONTR - construct a controller object for the Convex
%                     Interpolation Controller
%
% Syntax:
%       obj = OBJCONVINTERCONTR(dyn,Rfin,contrLaw,Param)
%       obj = OBJCONVINTERCONTR(dyn,Rfin,contrLaw,Param,occSet)
%
% Description:
%       Constructor of the controller object for the Convex Interpolation 
%       Controller. The object stores all computed data from the
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
%           -.controller:   used controller ('linear', 'quadratic', or 
%                           'exact')
%           -.Param:        cell-array storing the control law parameter
%                           for each time step
%           -.Nc:           number of center trajectory time steps
%           -.Ninter:       number of intermediate time steps
%           -.P:            cell-array storing a parallelotope enclosure of
%                           the reachable set for each time step    
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
%
%       -occSet:  cell-array storing the occupancy set  
%
% Output Arguments:
%
%       -obj:   resulting object of class objConvInterContr
%
% See Also:
%       convexInterpolationControl
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
    
properties (SetAccess = protected, GetAccess = public)
    controller = [];        % applied controller
    Nc = [];                % number of center trajectory time steps
    Ninter = [];            % number of intermediate time steps    
    controlLawParam = [];   % control law parameter
    parallelo = [];         % parallelotope enclosure of reachable set
end
   
methods

    function obj = objConvInterContr(dyn,Rfin,contrLaw,Param,varargin)
    % class constructor
    
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param,varargin{:});
        
        % store control law parameters                       
        obj.controller = contrLaw.controller;
        obj.Nc = contrLaw.Nc;
        obj.Ninter = contrLaw.Ninter;
        obj.controlLawParam = contrLaw.Param;
        obj.parallelo = contrLaw.P;
    end
end
end