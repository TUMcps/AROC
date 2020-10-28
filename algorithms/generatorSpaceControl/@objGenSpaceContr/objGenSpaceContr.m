classdef objGenSpaceContr < objController
% OBJGENSPACECONTR - construct a controller object for the synthesis
%                     algorithm based on optimal control in generator 
%                     space
%
% Syntax:
%       obj = OBJGENSPAECONTR(dyn,Rfin,contrLaw,Param)
%       obj = OBJGENSPAECONTR(dyn,Rfin,contrLaw,Param,occSet)
%
% Description:
%       Constructor of the controller object for the synthesis algorithm 
%       based on optimal control in generator space. The object stores all 
%       computed data from the offline-phase of the algorithm. The 
%       controller object can be used to simulate the online-phase of the 
%       algorithm.
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
%           -.Ninter:   number of intermediate time steps
%           -.alpha:    cell-array storing the factors alpha of the control
%                       law for each intermediate time step
%           -.P:        cell-array storing a parallelotope enclosure of the 
%                       reachable set for each time step    
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
%       - obj:      resulting object of class objGenSpaceContr
%
% See Also:
%       generatorSpaceControl
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
% Authors:      Jan Wagener, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    N = [];             % number of time steps
    Ninter = [];        % number of intermediate time steps
    alpha = [];         % factors alpha for the control law
    parallelo = [];     % parallelotope enclosure of time point reach. set
end
   
methods
    
    function obj = objGenSpaceContr(dyn,Rfin,contrLaw,Param,varargin)
    % class constructor   
       
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param,varargin{:});
    
        % store control law parameter                        
        obj.N = contrLaw.N;
        obj.Ninter = contrLaw.Ninter;
        obj.alpha = contrLaw.alpha;
        obj.parallelo = contrLaw.P;
        
        % convert set of admissible control inputs to zonotope
        obj.U = zonotope(obj.U);
    end
end
end