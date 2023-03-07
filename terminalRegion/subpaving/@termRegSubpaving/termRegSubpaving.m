classdef termRegSubpaving < terminalRegion
% TERMREGSUBPAVING - terminal region object computed with the subpaving
%                    algorithm
%
% Syntax:
%       obj = TERMREGSUBPAVING(dyn,set,K,subpav,xEq,uEq,Param)
%
% Description:
%       This class represents a terminal region computed with the subpaving
%       algorithm in [1]. The object can be used to simulate the controller
%       for the terminal region and can be provides as a terminal region
%       for model predictive control.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -set:       set representing the terminal region (class: contSet) 
%       -K:         feedback matrix for the terminal controller
%       -subpav:    cell-arry of intervals defining the subpaving that 
%                   represents the terminal region 
%       -xEq:       equilibrium point for the terminal region
%       -uEq:       control input for the equilirium point
%       -Param:     a structure containing the benchmark parameters
%
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval)
%
% Output Arguments:
%
%       -obj:   generated object of class termRegSubpaving
%
% See Also:
%       terminalRegion, compTermRegSubpaving
%
% References:
%       * *[1] El-Guindy et al. (2017)*, Estimating the region of 
%              attraction via forward reachable sets, ACC 2017
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    K = [];             % feedback matrix for the terminal controller
    subpaving = [];     % subpaving representing the terminal region
    xEq = [];           % equilibrium point for the terminal region
    uEq = [];           % control input for the equilibrium point
end
   
methods
    
    function obj = termRegSubpaving(dyn,set,K,subpav,xEq,uEq,Param)
    % class constructor   
    
        % call superclass constructor
        obj = obj@terminalRegion(dyn,set,Param);
        
        % store control law parameters                       
        obj.K = K;
        obj.subpaving = subpav;
        obj.xEq = xEq;
        obj.uEq = uEq;
    end   
end
end