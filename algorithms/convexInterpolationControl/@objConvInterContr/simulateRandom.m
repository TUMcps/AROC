function [res,t,x,u] = simulateRandom(obj,varargin)
% SIMULATERANDOM - simulate random trajectories of a nonlinear system 
%                  controlled with the Convex Interpolation Controller
%
% Syntax:
%       [res,t,x,u] = SIMULATERANDOM(obj)
%       [res,t,x,u] = SIMULATERANDOM(obj,Npoints,fracVert,fracDistVert,distChanges)
%
% Description:
%       Simulate a trajectories of a nonlinear closed-loop system 
%       controlled with the Convex Interpolation Controller for multiple
%       random initial points and randomly drawn disturbance values.
%
% Input Arguments:
%
%       -obj:           object of class objConvInterContr storing the 
%                       control law computed in the offline-phase
%       -res:           existing results object to which the simulation
%                       results should be added
%       -Npoints:       number of initial points for which trajectories are
%                       simulated
%       -fracVert:      fraction of random initial points corresponding to 
%                       vertices of the initial set (value in [0,1])
%       -fracDistVert:  fraction of random disturbance values corresponding
%                       to vertices of the disturbance set (value in [0,1])
%       -distChanges:   number of changes of the disturbance values during
%                       one time step of the algorithm (integer > 0)
%
% Output Arguments:
%       -res:   results object storing the simulation data
%       -t:     cell-array storing the vectors with the time points for all 
%               simulated trajectories
%       -x:     cell-array storing the matrices with all simulated 
%               trajectories
%       -u:     cell-array storing the applied control inputs for all
%               simulated trajectories
%
% See Also:
%       convexInterpolationControl, simulate
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
    [Npoints,fracVert,fracDistVert,distChanges] = ...
                                        defaultSimulateRandom(varargin{:});

    % determine initial points
    m = ceil(fracVert*Npoints);
    X0 = [randPoint(obj.R0,m,'extreme'),randPoint(obj.R0,Npoints-m)];

    % construct disturbance vectors
    W = cell(Npoints,1);
    V = cell(Npoints,1);
    Nw = obj.N*obj.Ninter*distChanges;
    Nextr = floor(Nw*fracDistVert);
    Wzono = zonotope(obj.W);
    
    for i = 1:Npoints
        
        % determine the indices of the extreme disturbance points
        if Nextr < Nw/2
            indExt = [];
            while size(indExt,1) ~= Nextr
              indExt = unique([indExt ; randi([1,Nw])]);
            end
            ind = setdiff(1:Nw,indExtr);
        else
            ind = [];
            while size(ind,1) ~= (Nw-Nextr)
              ind = unique([ind ; randi([1,Nw])]);
            end
            indExt = setdiff(1:Nw,ind);
        end
        
        % draw random points from the set of disturbances
        W{i} = zeros(dim(obj.W),distChanges);
        W{i}(:,indExt) = randPoint(Wzono,length(indExt),'extreme');
        W{i}(:,ind) = randPoint(Wzono,length(ind));
        
        % draw random measurement errors from the set of measurement errors
        V{i} = zeros(obj.nx,obj.N);
        
        if ~isempty(obj.V)
           V{i} = randPoint(obj.V,obj.N);
        end
    end
 
    % simulate trajectories
    x = cell(Npoints,1); u = cell(Npoints,1); t = cell(Npoints,1);
    sim = {};
    
    for i = 1:Npoints
       [~,t{i},x{i},u{i}] = simulate(obj,X0(:,i),W{i},V{i});
       sim{end+1}.t = t{i};
       sim{end}.x = x{i};
       sim{end}.u = u{i};
    end
    
    res = results([],[],[],sim);
end