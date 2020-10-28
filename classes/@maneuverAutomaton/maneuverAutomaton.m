classdef maneuverAutomaton
% MANEUVERAUTOMATON - class representing a Maneuver Automaton
%
% Syntax:
%       obj = MANEUVERAUTOMATON(primitives,shiftFun,shiftOccFun)
%
% Description:
%       This class represents a Maneuver Automaton. The maneuver automaton
%       is constructed from multiple motion primives computed with one of
%       the control algorithms in the AROC toolbox. Afterwards, the
%       automaton can be used to solve online control problems
%
% Input Arguments:
%
%       -primitives:    cell-array storing the motion primitives. Each
%                       motion primitive is an object of class 
%                       "objController"
%       -shiftFun:      function handle to a function that shifts the 
%                       initial set of a motion primitive to the final 
%                       state of the previous primitive 
%       -shiftOccFun:   function handle to a function that shifts the 
%                       occupancy set of a motion primitive to the final 
%                       state of the previous primitive
%
% Output Arguments:
%
%       -obj:   generated object of class maneuverAutomaton
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    
    
properties (SetAccess = protected, GetAccess = public)
    primitives = [];      % cell-array storing the motion primitives
    shiftFun = [];        % function handle to initial set shift function
    shiftOccFun = [];     % function handle to occupancy shift function
    conMat = [];          % connectivity matrix
end
   
methods
    
    function obj = maneuverAutomaton(primitives,shiftFun,shiftOccFun)
    % class constructor   
    
        % store input arguments
        obj.primitives = primitives;
        obj.shiftFun = shiftFun;
        obj.shiftOccFun = shiftOccFun;
        
        % construct connectivity matrix
        obj.conMat = connectivityMatrix(obj);
    end   
    
    function conMat = connectivityMatrix(obj)
    % construct the connectivity matrix that stores which maneuver can be
    % connected to which other maneuver:
    %
    % conMat(i,j) = 1 => maneuver j can be executed after manuever i
    
        conMat = zeros(length(obj.primitives));
        
        for i = 1:length(obj.primitives)
            for j = 1:length(obj.primitives)
               
                % get final state and final set of previous maneuver
                xf = obj.primitives{i}.xf;
                Rfin = obj.primitives{i}.Rfin;
                
                % compute shifted initial set for next maneuver
                Rinit = obj.primitives{j}.R0;
                Rinit = obj.shiftFun(Rinit,xf);
                
                % check if final set is located in shifted initial set
                conMat(i,j) = in(mptPolytope(Rinit),Rfin);
            end
        end
    end
    
    function xNew = updateState(obj,x,i)
    % this function computes the new state after execution of the i-th 
    % maneuver  
    
        % compute difference between initial and final state of maneuver
        xf = obj.primitives{i}.xf;
        x0 = center(obj.primitives{i}.R0);
        
        diff = zonotope([x0,xf-x0]);
        
        % compute updated state
        Rnew = obj.shiftFun(diff,x);
        xNew = center(Rnew) + generators(Rnew);
    end
    
    function occSet = updateOccupancy(obj,x,i,time)
    % this function transforms the occupancy set of the i-th maneuver 
    % considering the current state x 
    
        occSet = obj.primitives{i}.occupancySet;
        occSet = obj.shiftOccFun(occSet,x,time);
    end
end
end