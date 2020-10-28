classdef results
% RESULTS - class storing the reachalbe set and/or simulation of the
%           computted control law
%
% Syntax:
%       obj = RESULTS(reachSet,reachSetTimePoint,refTraj)
%       obj = RESULTS(reachSet,reachSetTimePoint,refTraj,simulation)
%
% Description:
%       Constructor of the results objects that stores the computed
%       reachable set, the reference trajectory and/or simulated
%       trajectories for the control law.
%
% Input Arguments:
%
%       -reachSet:              reachable set for the computed maneuver
%                               (class: reachSet)
%       -reachSetTimePoint:     reachable set for the computed maneuver at
%                               the begging of each center trajectory time 
%                               step
%       -refTraj:               reference trajectory for the maneuver    
%       -simulation:            data from the simulation of the system 
%
% Output Arguments:
%       -obj:   resulting object of class results
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
    reachSet = [];
    reachSetTimePoint = [];
    refTraj = [];
    simulation = [];
end
   
methods

    function obj = results(reachSet,reachSetTimePoint,refTraj,varargin)
    % class constructor
           
        % store input arguments in properties                         
        obj.reachSet = reachSet;
        obj.reachSetTimePoint = reachSetTimePoint;
        obj.refTraj = refTraj;
        
        if nargin > 3
            obj.simulation = varargin{1};
        else
            obj.simulation = []; 
        end
    end
    
    function points = getFinalSimPoints(obj)
    % get the final points from the system simulations   
           
        points = [];
    
        if ~isempty(obj.simulation)
            
            % loop over all simulated trajectories
            for i = 1:length(obj.simulation)
                
                % get the final point of the simulation
                xFin = obj.simulation{i}.x(end,:)';
                points = [points,xFin];
            end
        end
    end
    
    function res = checkFinalInInitSet(obj,R0)
    % checks if the final reachable set if contained inside the shifted
    % initial set
    
        % get initial and final reachable set
        Rfin = obj.reachSetTimePoint{end};
        
        % shift the initial set
        R0 = R0 + (-center(R0)) + center(Rfin);
        
        % check set containment
        res = in(R0,Rfin);
    end
    
    function res = checkFinalSimPoints(obj)
    % checks if the final points of the simulated trajectories are located 
    % inside the final reachable set   
        
        res = 1;
    
        % get final points of simulations
        points = getFinalSimPoints(obj);
        
        % get the final reachable set
        R = obj.reachSetTimePoint{end};
        
        % check if the points are contained
        if isa(R,'polyZonotope')
           res = containsPointSet(R,points,3,2);
        else
           % loop over all points
           for i = 1:size(points,2)
              if ~in(R,points(:,i))
                  res = 0;
                  return;
              end
           end
        end 
    end
    
    function res = checkSimInputs(obj,U)
    % checks if the control inputs from all simulations satisfy the input 
    % constraints  
        
        res = 1;
        tol = 1e-10;
        
        % convert set of admissble control inputs to a polytope
        poly = mptPolytope(U);
        C = get(poly,'A');
        d = get(poly,'b');
        
        if ~isempty(obj.simulation)
           
            % loop over all simulated trajectories
            for i = 1:length(obj.simulation)
               
                u = obj.simulation{i}.u;
                
                % loop over all points of the simulation
                for j = 1:size(u,1)
                   
                    % check if u is contained in the set of admissble
                    % control inputs
                    if ~all(C*u(j,:)'-d < tol)
                        res = 0;
                        return;
                    end
                end
            end
        end
    end
    
    function res = checkFinalSet(obj,Rfin)
    % checks if the final reachable set is identical to the stored result
    
        % get final reachable set
        Rfin_ = obj.reachSetTimePoint{end};
        
        % interval encloure
        Rfin_ = interval(Rfin_);
        
        % check set containment
        fac = 1 + 1e-4;
        res = in(enlarge(Rfin_,fac),Rfin) & in(enlarge(Rfin,fac),Rfin_);
    end
    
    function res = checkInitialSet(obj,Rinit)
    % checks if the initial reachable set is identical to the stored result
    
        % get final reachable set
        Rinit_ = obj.reachSetTimePoint{1};
        
        % interval encloure
        Rinit_ = interval(Rinit_);
        
        % check set containment
        fac = 1 + 1e-4;
        res = in(enlarge(Rinit_,fac),Rinit) & in(enlarge(Rinit,fac),Rinit_);
    end
    
end
end