classdef terminalRegion
% TERMINALREGION - abstract class for all terminal regions in AROC
%
% Syntax:
%       obj = TERMINALREGION(dyn,set,Param)
%
% Description:
%       This class is the superclass for all terminal region objects in the 
%       AROC toolbox. The class defines some common properties that are
%       inherited by all terminal region objects.
%
% Input Arguments:
%
%       -dyn:       function handle to the dynamic function of the 
%                   open-loop system
%       -set:       set representing the terminal region (class: contSet) 
%       -Param:     a structure containing the benchmark parameters
%
%           -.U:        set of admissible control inputs 
%                       (class: interval or zonotope)
%           -.W:        set of uncertain disturbances 
%                       (class: interval or zonotope)
%           -.V:        set of measurement errors 
%                       (class: interval or zonotope)
%           -.X:        set of state constraints (class: mptPolytope)
%
% Output Arguments:
%
%       -obj:   generated object of class terminalRegion
%
% See Also:
%       termRegSubpaving, termRegZonoLinSys
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
    dynamics = [];      % function handle to the dynamic function
    set = [];           % set representing the terminal region
    U = [];             % set of admissble control inpus
    W = [];             % set of disturbances
    V = [];             % set of measurement errors
    X = [];             % set of state constraints
end
   
methods
    
    function obj = terminalRegion(dyn,set,Param)
    % class constructor   
    
        obj.dynamics = dyn;
        obj.set = set;
        obj.U = Param.U;
        obj.W = Param.W;
        
        if isfield(Param,'V')
            obj.V = Param.V;
        else
            obj.V = []; 
        end
        
        if isfield(Param,'X')
            obj.X = Param.X;
        else
            obj.X = [];
        end

    end   
end
end