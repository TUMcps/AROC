function [res,t,xSim,uSim] = simulate(obj,x0,tFinal,w)
% SIMULATE - simulate a trajectory of a terminal region controller
%           
% Syntax:
%       [res,t,xSim,uSim] = SIMULATE(obj,x0,tFinal,w)
%
% Description:
%       Simulate a trajectory of a linear closed-loop system controlled
%       with the terminal region controller corresponding to a terminal
%       region computed with the zonotope approach for linear systems.
%
% Input Arguments:
%       -obj:       instance of class computeTermRegNonlinSysOpt
%       -x0:        initial state for simulation
%       -tFinal:    final time for simulation
%       -w:         array storing the disturbance trajectory
%                   (double arry of dimension: [nw,timeSteps])
%
% Output Arguments:
%
%       -res:   results object storing the simulation data
%       -t:     vector storing the time points for the simulated states
%       -xSim:  array storing the simulated trajectory 
%               (double array of dimension: [|t|,nx])
%       -uSim:  array storing the applied control input 
%               (double array of dimension: [|t|,nu])
%
% See Also:
%       computeTerminalRegionNonlinSys, computeTermRegNolinSysOptApproach,
%       @termRegNolinSysOptApproach/simulateRandom
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

% initialization 
nx = length(x0);
% ... time grid
if obj.sysType.isDiscreteTimeSystem
    t_events = 0:obj.dt:tFinal;
    t_w = t_events; t_control = t_events;
else
    t_w  = linspace(0,tFinal,size(w,2)+1);
    t_control = 0:obj.dt:tFinal;
    t_events = sort(unique([t_control, t_w]));
end
kk_u = 1;
kk_w = 1;

% simulate
xSim = x0'; uSim = []; t = [];
x0_ = x0;
for kk = 1:length(t_events)-1

    t0 = t_events(kk);
    t1 = t_events(kk+1);

    % update control if needed
    if withinTol(t0, t_control(kk_u),1e-8)
        u = obj.computeControlInput(x0_);
        kk_u= kk_u + 1;
    end

    % update disturbance if needed
    if withinTol(t0, t_w(kk_w),1e-8)
        w_ = w(:,kk_w);
        kk_w = kk_w + 1;
    end

    % propagate system
    if obj.sysType.isDiscreteTimeSystem
        xTemp = obj.dynamics(x0_,u,w_)';
        tTemp = [t0; t1];
    else
        [tTemp,xTemp] = ode45(@(t,x)obj.dynamics(x,u,w_), ...
                                            [t0, t1],x0_);
    end

    % store simulation results
    xSim = [xSim;xTemp(:,1:nx)];
    uSim = [uSim;ones(size(xTemp,1),1)*u'];
    t = [t;tTemp];
    x0_ = xSim(end,:)';
end

% store simulation in results object
sim{1}.t = t;
sim{1}.x = xSim;
sim{1}.u = uSim;
res = results([],[],[],sim);

end
