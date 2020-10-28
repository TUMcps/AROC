classdef objController
% OBJCONTROLLER - abstract class for all controllers in AROC
%
% Syntax:
%       obj = OBJCONTROLLER(dyn,Rfin,Param)
%       obj = OBJCONTROLLER(dyn,Rfin,Param,occSet)
%
% Description:
%       This class is the superclass for all controller objects in the AROC
%       toolbox. The class defines some common properties that are
%       inherited by all controller objects.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -Rfin:      final reachable set
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
%       -obj:   generated object of class objController
%
% See Also:
%       objConvInterContr, objSafetyNetContr, objGenSpaceContr
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
% Authors:      Niklas Kochdumper, Moritz Klischat
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    dynamics = [];      % function handle to the dynamic function
    R0 = [];            % initial set
    Rfin = [];          % final reachable set
    xf = [];            % goal state
    tFinal = [];        % final time
    U = [];             % set of admissble control inpus
    W = [];             % set of disturbances
    occupancySet = [];  % cell-array storing the occupancy set      
    nx = [];            % number of states
    nu = [];            % number of inputs
    nw = [];            % number of disturbances
end
   
methods
    
    function obj = objController(dyn,Rfin,Param,varargin)
    % class constructor   
    
        % assign object properties
        obj.dynamics = dyn;
        obj.Rfin = Rfin;
        
        obj.R0 = Param.R0;
        obj.xf = Param.xf;
        obj.tFinal = Param.tFinal;
        obj.U = Param.U;
        obj.W = Param.W;
        
        % store occupancy set if provided
        if nargin > 3
           obj.occupancySet = varargin{1}; 
        end
        
        % store useful paramter
        obj.nx = dim(obj.R0);
        obj.nu = dim(obj.U);
        obj.nw = dim(obj.W);
    end   
end
end