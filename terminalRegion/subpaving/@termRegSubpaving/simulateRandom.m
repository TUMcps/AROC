function [res,t,x,u] = simulateRandom(obj,tFinal,varargin)
% SIMULATERANDOM - simulate random trajectories of a nonlinear system 
%                  controlled with a terminal region controller
%
% Syntax:
%       [res,t,x,u] = SIMULATERANDOM(obj,tFinal)
%       [res,t,x,u] = SIMULATERANDOM(obj,tFinal,Npoints,fracVert,fracDistVert,distChanges)
%
% Description:
%       Simulate a trajectories of a nonlinear closed-loop system 
%       controlled with a terminal region controller corresponding to a 
%       terminal region computed with the subpaving algorithm for
%       random initial points inside the terminal region and randomly drawn 
%       disturbance values.
%
% Input Arguments:
%
%       -obj:           object of class termRegSubpaving storing the 
%                       control law computed for the terminal region 
%                       controller.
%       -tFinal:        final time for the simulation
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
%       compTermRegSubpaving, termRegSubpaving, simulate
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

    % parse user inputs
    [Npoints,fracVert,fracDistVert,distChanges] = ...
                                        defaultSimulateRandom(varargin{:});

    % determine initial points
    m = ceil(fracVert*Npoints);
    X0 = [randPoint(obj.set,m,'extreme'),randPoint(obj.set,Npoints-m)];

    % construct disturbance vectors
    W = cell(Npoints,1);
    V = cell(Npoints,1);
    Nextr = floor(distChanges*fracDistVert);
    Wzono = zonotope(obj.W);
    
    for i = 1:Npoints
        
        % determine the indices of the extreme disturbance points
        indExt = [];
        while size(indExt,1) ~= Nextr
          indExt = unique([indExt ; randi([1,distChanges])]);
        end
        ind = setdiff(1:distChanges,indExt);
        
        % draw random points from the set of disturbances
        W{i} = zeros(dim(obj.W),distChanges);
        W{i}(:,indExt) = randPoint(Wzono,length(indExt),'extreme');
        W{i}(:,ind) = randPoint(Wzono,length(ind));
        
        % draw random measurement errors from the set of measurement errors
        V{i} = zeros(dim(obj.set),distChanges);
        
        if ~isempty(obj.V)
            V{i} = randPoint(obj.V,distChanges); 
        end
    end
 
    % simulate trajectories
    x = cell(Npoints,1); u = cell(Npoints,1); t = cell(Npoints,1);
    sim = {};
    
    for i = 1:Npoints
       [~,t{i},x{i},u{i}] = simulate(obj,X0(:,i),tFinal,W{i},V{i});
       sim{end+1}.t = t{i};
       sim{end}.x = x{i};
       sim{end}.u = u{i};
    end
    
    res = results([],[],[],sim);
end