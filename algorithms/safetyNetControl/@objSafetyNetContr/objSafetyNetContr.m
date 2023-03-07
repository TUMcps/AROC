classdef objSafetyNetContr < objController
% OBJSAFETYNETCONTR - construct a controller object for the Safety Net
%                     Controller
%
% Syntax:
%       obj = OBJSAFETYNETCONTR(dyn,Rfin,contrLaw,Param)
%       obj = OBJSAFETYNETCONTR(dyn,Rfin,contrLaw,Param,occSet)
%
% Description:
%       Constructor of the controller object for the Safety Net 
%       Controller. The object stores the safe sets and input weights for
%       the safety net controller.  The controller object can be used 
%       to simulate the online-phase of the algorithm. Furthermore, a 
%       controller for the unverified controller is stored which can be 
%       simulated in combination with the safety net.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -Rfin:      final reachable set
%
%       -contrLaw:  a structure containing the control law parameter
%
%           -.comfContr:    controller object for the comfort controller 
%           -.H:            cell-array storing the input-to-generator
%                           correlation matrix for each time step
%           -.reachSet:     cell-array storing the time point reachable set
%           -.Usub:         cell-array storing the subsets of admissble
%                           control inputs for each time step
%           -.N:            number of time steps
%           -.Ninter:       number of intermediate time steps 
%           -.realTime:     flag specifying if controller is run in
%                           real-time mode or not
%           -.tComp:        allocated computation time
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
%
%       -occSet:  cell-array storing the occupancy set 
%
% Output Arguments:
%
%       -obj:   resulting object of class objSafetyNetContr
%
% See Also:
%       safetyNetControl
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    
    comfContr = [];     % comfort controller object
    H = [];             % generator-to-input correlation matrix
    reachSet = [];      % time point reachable set
    Usub = [];          % subsets of admissible control inputs
    N = [];             % number of time steps
    Ninter = [];        % number of intermediate time steps
    Npred = [];         % number of time step during alloc. comp. time
    Rinit = [];         % shifted goal set
    tComp = [];         % allocated computation time for comfort controller
    realTime = [];      % flag specifying if run in real-time mode or not
    sysPred = [];       % system dynamics for prediction
end
   
methods

    function obj = objSafetyNetContr(dyn,Rfin,contrLaw,Param,varargin)
    % class constructor
                     
        % call superclass constructor
        obj = obj@objController(dyn,Rfin,Param,varargin{:});
    
        % store control law parameter
        if ~iscell(contrLaw.comfContr)
            obj.comfContr = {contrLaw.comfContr};
        else
            obj.comfContr = contrLaw.comfContr;
        end
        obj.H = contrLaw.H;
        obj.Usub = contrLaw.Usub;
        obj.reachSet = contrLaw.reachSet;
        obj.N = contrLaw.N;
        obj.Ninter = contrLaw.Ninter; 
        obj.Rinit = contrLaw.Rinit;
        obj.realTime = contrLaw.realTime;
        obj.tComp = contrLaw.tComp;
        
        % set-up variables required for prediction of system state at the
        % end of the allocated computation time
        obj.Npred = obj.tComp/(Param.tFinal/(obj.N*obj.Ninter))+1;
        
        fun = @(x,w) dynamicsClosedLoopLinear(x,w,obj.nx,obj.nu,dyn);
        name = 'AROCsafetyNetPred';
                                           
        obj.sysPred = nonlinearSys(name,@(x,w) fun(x,w), ...
                                   obj.nx+obj.nu,obj.nw); 
    end
end
end