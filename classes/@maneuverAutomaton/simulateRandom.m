function [res,t,x,u] = simulateRandom(obj,ind,x0,varargin)
% SIMULATERANDOM - simulate random trajectories of a nonlinear system 
%                  controlled with a maneuver automaton
%
% Syntax:
%       [res,t,x,u] = SIMULATERANDOM(obj,ind,x0)
%       [res,t,x,u] = SIMULATERANDOM(obj,ind,x0,Npoints,fracDistVert,distChanges)
%
% Description:
%       Simulate trajectories of a nonlinear closed-loop system 
%       controlled with a maneuver automaton for randomly drawn disturbance 
%       values.
%
% Input Arguments:
%
%       -obj:           object of class maneuverAutomaton storing the 
%                       control law computed in the offline-phase
%       -ind:           indices of the motion primitives for the planned 
%                       trajectory
%       -x0:            initial point for the simulation
%       -Npoints:       number of trajectories that are simulated
%       -fracDistVert:  fraction of random disturbance values corresponding
%                       to vertices of the disturbance set (value in [0,1])
%       -distChanges:   number of changes of the disturbance values during
%                       one time step of the algorithm (integer > 0)
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     cell-array storing the vectors with the time points for all 
%               simulated trajectories
%       -x:     cell-array storing the matrices with all simulated 
%               trajectories
%       -u:     cell-array storing the matrices with the applied control 
%               inputs for all simulated trajectories
%
% See Also:
%       maneuverAutomaton, simulate
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

    % parse user inputs
    [Npoints,~,fracDistVert,distChanges] = ...
                                        defaultSimulateRandom(varargin{:});

    % construct disturbance vectors 
    W = cell(Npoints,1);
    V = cell(Npoints,1);
   
    for i = 1:Npoints

        W_ = cell(length(ind),1);
        V_ = cell(length(ind),1);

        % loop over all motion primitives
        for j = 1:length(ind)

            % number of disturbances and measurement errors for controller
            objContr = obj.primitives{ind(j)};
            [Nw,Nv] = numberDistMeasErr(objContr,distChanges);
        
            % determine the indices of the extreme disturbance points
            Nextr = floor(Nw*fracDistVert);

            if Nextr < Nw/2
                indExt = [];
                while size(indExt,1) ~= Nextr
                  indExt = unique([indExt ; randi([1,Nw])]);
                end
                index = setdiff(1:Nw,indExtr);
            else
                index = [];
                while size(index,1) ~= (Nw-Nextr)
                  index = unique([index ; randi([1,Nw])]);
                end
                indExt = setdiff(1:Nw,index);
            end
            
            % draw random points from the set of disturbances
            Wzono = zonotope(objContr.W);

            W_{j} = zeros(dim(objContr.W),Nw);
            W_{j}(:,indExt) = randPoint(Wzono,length(indExt),'extreme');
            W_{j}(:,index) = randPoint(Wzono,length(index));
            
            % draw random measurement errors from the corresponding set
            V_{j} = zeros(objContr.nx,Nv);
            
            if ~isempty(objContr.V)
                V_{j} = randPoint(objContr.V,Nw);
            end
        end

        W{i} = W_; V{i} = V_;
    end
 
    % simulate trajectories
    x = cell(Npoints,1); u = cell(Npoints,1); t = cell(Npoints,1);
    sim = {};
    
    for i = 1:Npoints
       [~,t{i},x{i},u{i}] = simulate(obj,ind,x0,W{i},V{i});
       sim{end+1}.t = t{i};
       sim{end}.x = x{i};
       sim{end}.u = u{i};
    end
    
    res = results([],[],[],sim);
end


% Auxiliary Functions -----------------------------------------------------

function [Nw,Nv] = numberDistMeasErr(obj,distChanges)
% get number of disturbances and measurement errors for the controller

    if isa(obj,'objGenSpaceContr') || isa(obj,'objConvInterContr') ...
                                        || isa(obj,'objPolyContr')

        Nw = obj.N * obj.Ninter * distChanges;
        Nv = obj.N;

    elseif isa(obj,'objOptBasedContr') || isa(obj,'objCombinedContr')

        Nw = obj.N * distChanges;
        Nv = obj.N * distChanges;

    elseif isa(obj,'objSafetyNetContr')

        Nw = obj.N * obj.Ninter * distChanges;
        Nv = obj.N * obj.Ninter * distChanges;

    else
        error('This controller is not supported!');
    end
end