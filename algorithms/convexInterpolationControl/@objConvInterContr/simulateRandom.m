function [res,t,x,u] = simulateRandom(obj,res,Npoints,fracVert,fracDistVert,distChanges)
% SIMULATERANDOM - simulate random trajectories of a nonlinear system 
%                  controlled with the Convex Interpolation Controller
%
% Syntax:
%       [res,t,x,u] = SIMULATERANDOM(obj,res,Npoints,fracVert,fracDistVert,distChanges)
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

    % determine initial points
    X0 = zeros(obj.nx,Npoints);

    for i = 1:Npoints
       if i < Npoints*fracVert
          X0(:,i) = randPointExtreme(obj.parallelo{1});
       else
          X0(:,i) = randPoint(obj.parallelo{1});
       end
    end

    % construct disturbance vectors
    W = cell(Npoints,1);
    Nw = obj.Nc*obj.Ninter*distChanges;
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
        W{i} = zeros(obj.nw,Nw);
        
        for j = 1:length(indExt)
            W{i}(:,indExt(j)) = randPointExtreme(Wzono);
        end
        
        for j = 1:length(ind)
            W{i}(:,ind(j)) = randPoint(Wzono);
        end
    end
 
    % simulate trajectories
    x = cell(Npoints,1);
    u = cell(Npoints,1);
    t = cell(Npoints,1);
    
    for i = 1:Npoints
       [res,t{i},x{i},u{i}] = simulate(obj,res,X0(:,i),W{i});
    end

end