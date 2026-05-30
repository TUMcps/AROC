function xNext = RK2integrator(x0,u,w,funHandle,dt)
%RK2INTEGRATOR  - implements the midpoint method for numerically solving 
%       ordinary differential equations.
%
% Syntax:
%       xNext = RK4integrator(x0,u,w,funHandle,dt)
%
% Description:
%       This function implements the midpoint method, a second-order 
%       Runge-Kutta method with two stages and, in addition to solving the 
%       initial value problem
%           x_dot = f(x,u,w),  x(0) = x0
%       numerically, can be used for time-discretizing continuous-time 
%       dynamical systems, e.g. for reachability analysis using CORA. Both 
%       the control input "u" and the disturbance "w" are assumed to be 
%       constant during the time step.
%
% Input Arguments:
%       -x0:            initial state
%                       (real array of dimension [nx,1])
%       -u:             control input
%                       (real array of dimension [nu,1])
%       -w:             disturbance
%                       (real array of dimension [nw,1])
%       -funHandle:     function handle to the dynamics function of the
%                       continuous-time system 
%                       (x_dot = f(x,u,w)
%       -dt:            time step
%                       (positive scalar)
%
% Output Arguments:
%       -xNext:     state at the end of the time interval [0, dt]
%                   (real array of dimension [nx,1])
%
%
% See Also:
%       -
% 
% References:
%       [1] W. Kutta, "Beitrag zur näherungsweisen Integration totaler 
%           Differentialgleichungen", Zeitschrift für Mathematik 
%           und Physik, 46: 435–453, 1901.
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
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

k1 = funHandle(x0,u,w);
k2 = funHandle(x0+0.5*dt*k1,u,w);

xNext = x0 + dt*k2;

end
