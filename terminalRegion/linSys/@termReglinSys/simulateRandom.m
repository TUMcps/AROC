function trajectories = simulateRandom(T, Npoints, samplingMethod)
% SIMULATERANDOM - simulate random trajectories of a linear system
%                  controlled with a safe set controller
%
% Syntax:
%       trajectories = SIMULATERANDOM(obj, Npoints, sampleMethod)
%
% Description:
%       Simulate trajectories of a linear time-invariant closed-loop system
%       controlled with a terminal region controller corresponding to a
%       terminal region computed with the linSysContainment algorithm for
%       random initial points inside the terminal region and randomly drawn
%       disturbance values.
%
% Input Arguments:
%
%       -Npoints:        number of initial points for which trajectories
%                        are simulated
%       -samplingMethod: method used for sampling points, for example
%                       'extreme' or 'uniform' 
%
% Output Arguments:
%       -trajectories:   cell with Npoints entries, each storing the data
%                        of a simulated trajectory (see simulate.m) 
%
% See Also:
%       safeSets/simulateRandom
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
% Authors:      Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% Check arguments (in this case, only check Npoints)
inputArgsCheck({{Npoints,'att','numeric',{'scalar','vector','nonnan','finite','positive'}}});

% Generate some points in the terminal region
termReg = T.set;
y0 = termReg.randPoint(Npoints, samplingMethod);

trajectories = cell([1 Npoints]);

for i = 1:Npoints
    % Generate random disturbances
    w = T.W.randPoint(T.Opts.N, samplingMethod);
    v = T.V.randPoint(T.Opts.N+1, samplingMethod);

    trajectory = simulate(T,y0(:,i),w,v);


    trajectories{i} = trajectory;
end


