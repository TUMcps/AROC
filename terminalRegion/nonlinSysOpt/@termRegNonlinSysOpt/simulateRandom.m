function [res,t,x,u] = simulateRandom(obj,tFinal,varargin)
% SIMULATERANDOM - simulate random trajectories of a nonlinear system 
%       controlled with a terminal region controller
%
% Syntax:
%       [res,t,x,u] = SIMULATERANDOM(obj,tFinal)
%       [res,t,x,u] = SIMULATERANDOM(obj,tFinal,Npoints,fracVert,fracDistVert,distChanges)
%
% Description:
%       Simulate a trajectories of a nonlinear closed-loop system 
%       controlled with a terminal region controller corresponding to a 
%       zonotopic terminal region computed with the approaches from [1]/[2].
%
% Input Arguments:
%       -obj:           instance of class computeTermRegNonlinSysOpt
%       -tFinal:        final time for simulation
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
%
%       -res:   results object storing the simulation data
%       -t:     cell-array storing the vectors with the time points for all 
%               simulated trajectories
%       -x:     cell-array storing the matrices with all simulated 
%               trajectories
%       -u:     cell-array storing the applied control inputs for all
%               simulated trajectories
%
% See Also:
%       computeTerminalRegionNonlinSys, computeTermRegNolinSysOptApproach,
%       @termRegNolinSysOptApproach/simulateRandom
%
% References:
%       [1] L. Schäfer et al., "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770.
%       [2] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169. 
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------    

% parse user inputs
[Npoints,fracVert,fracDistVert,distChanges] = ...
                                    defaultSimulateRandom(varargin{:});

% parse user inputs for discrete-time systems
% ... number of time steps is assumed to be an integer
if mod(tFinal,obj.dt) ~= 0
    tFinal = floor(tFinal/obj.dt)*obj.dt;
    warning(['Discrete-time system requires number of time steps to ' ...
        'be an integer - set tFinal to %2f'], tFinal);
end
% ... disturbance is assumed to be constant during "sampling" interval
if obj.sysType.isDiscreteTimeSystem
    distChanges = 1;
end

% sample initial states
m = ceil(fracVert*Npoints);
X0 = [randPoint(obj.set,m,'extreme'),randPoint(obj.set,Npoints-m)];

% sampe disturbance trajectories -----------------------------------------
N = round(tFinal/obj.dt,0);
W = cell(Npoints,1);
Nw = N*distChanges;
Nextr = floor(Nw*fracDistVert);
Wzono = zonotope(obj.W);

for i = 1:Npoints
    
    % determine the indices of the extreme disturbance points
    if Nextr < Nw/2
        indExt = [];
        while size(indExt,1) ~= Nextr
          indExt = unique([indExt ; randi([1,Nw])]);
        end
        ind = setdiff(1:Nw,indExt);
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
end

% simulate trajectories --------------------------------------------------
x = cell(Npoints,1); u = cell(Npoints,1); t = cell(Npoints,1);
sim = {};

for ii = 1:Npoints
   [~,t{ii},x{ii},u{ii}] = simulate(obj,X0(:,ii),tFinal,W{ii});
   sim{end+1}.t = t{ii};
   sim{end}.x = x{ii};
   sim{end}.u = u{ii};
end
res = results([],[],[],sim);

end
