classdef objOptBasedContr < objController
% OBJOPTBASEDCONTR - construct a controller object for the Optimization 
%                    Based Control Algorithm
%
% Syntax:
%       obj = OBJOPTBASEDCONTR(dyn,Rfin,contrLaw,Param)
%       obj = OBJOPTBASEDCONTR(dyn,Rfin,contrLaw,Param,occSet)
%
% Description:
%       Constructor of the controller object for the Optimization Based
%       Control Algorithm. The object stores all computed data from the 
%       offline-phase of the algorithm. The controller object can be used 
%       to simulate the online-phase of the algorithm.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -Rfin:      final reachable set
%
%       -contrLaw:  a structure containing the control law parameter
%
%           -.N:        number of time steps
%           -.K:        cell-array storing the feedback matrices for all
%                       time steps
%           -.u_ref:    reference input for all time steps
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
%       -obj:   resulting object of class objOptBasedContr
%
% See Also:
%       optimizationBasedControl
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
% Authors:      Ivan Hernandez, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    
    isLin = 1;          % 1 if model is linear, 0 if not
    N = [];             % number of center trajectory time steps
    K = [];             % cell-array storing the feedback matrices
    u_ref = [];         % reference input of the center trajectory
end
   
methods

    function obj = objOptBasedContr(dyn,Rfin,contrLaw,Param,varargin)
        
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param,varargin{:});
    
        % store control law parameter
        obj.K = contrLaw.K;
        obj.u_ref = contrLaw.u_ref;
        obj.N = contrLaw.N;
        
        % store additional information        
        if isstruct(dyn)
           obj.isLin = 1; 
        else
           obj.isLin = 0;
        end                       
    end
end
end