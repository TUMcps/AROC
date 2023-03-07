classdef objPolyContr < objController
% OBJPOLYCONTR - construct a controller object for polynomial controller 
%                synthesis
%
% Syntax:
%       obj = OBJPOLYCONTR(dyn,Rfin,contrLaw,Param)
%
% Description:
%       Constructor of the controller object for the polynomial control 
%       synthesis algorithm. The object stores all computed data from the 
%       offline-phase of the algorithm. The controller object can be used 
%       to simulate the online-phase of the algorithm.
%
% Input Arguments: 
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -Rfin:      final reachable set
%       -contrLaw:  a structure containing the control law parameter
%
%           -.N:            number of time steps
%           -.Ninter:       number of intermediate time steps
%           -.Ctrl_x:       cell array of N, Ninter*nu dimensional 
%                           controller that take the state as argument  
%           -.parallelo:    cell array storing the parallelotope enclosures
%                           of the reachable set
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
%       -occSet:  cell-array storing the occupancy set  
%
% Output Arguments:
%       - obj:      resulting object of class objPolyContr
%
% See Also:
%       polynomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Arificial Intelligcen and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    N = [];             % number of time steps
    Ninter = [];        % number of intermediate time steps
    Ctrl_x = [];        % controller u(x)
    parallelo = [];     % parallelotope enclosures of the reachable set
end
   
methods
    
    function obj = objPolyContr(dyn,Rfin,contrLaw,Param,varargin)
    % class constructor   
       
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param,varargin{:});
    
        % store control law parameter                        
        obj.N = contrLaw.N;
        obj.Ninter = contrLaw.Ninter;
        obj.Ctrl_x = contrLaw.Ctrl_x;
        obj.parallelo = contrLaw.parallelo;

        % convert set of admissible control inputs to zonotope
        obj.U = zonotope(obj.U);
    end
end
end