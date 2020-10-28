function f = stirredTankReactor(x,u,w)
% STIRREDTANKREACTOR - dynamic equations for the stirred tank reactor 
%                      benchmark model
%
% Syntax:
%       f = STIRREDTANKREACTOR(x, u, w)
%
% Description:
%       Dynamic equation for the stirred tank reactor benchmark model. The 
%       benchmark models the exothermic and irreversal reaction of a
%       reactant A to a product B as described in [1]. The system states 
%       are the concentration of the reactant A and the temperature in the 
%       reactor. The system input is the temperature of the coolant.
%
%       <<img/stirredTankReactor.png>>
%
% Input Arguments:
%
%       -x:     system states x (dimension: [nx,1])
%       -u:     control inputs u (dimension: [nu,1]) 
%       -w:     disturbances w (dimension: [nw,1])
%
% Output Arguments:
%
%       -f:     value of the dynamic function = time derivative of x
%
% See Also:
%       cart, robotArm
%
% References:
%       * *[1] Magni et al. (2001)*, A stabilizing model-based predictive 
%              control algorithm for nonlinear systems
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

    % Parameter
    rho = 1000;
    Cp = 0.239;
    deltaH = -5e4;
    E_R = 8750;
    k0 = 7.2e10 / 60;
    UA = 5e4 / 60;
    q = 5/3;
    Tf = 350;
    V = 100;
    C_Af = 1;
    C_A0 = 0.5;
    T_0 = 350;
    T_c0 = 300;

    % State offset
    x = x + [C_A0;T_0];

    % Control law offset
    U = u + T_c0;

    % Dynamics
    f(1,1) = q/V * (C_Af - x(1)) - k0*exp(-E_R/x(2)) * x(1) + w(1);

    f(2,1) = q/V * (Tf - x(2)) - (deltaH*k0)/(rho*Cp)* exp(-E_R/x(2)) * x(1) + ...
             UA/(V*rho*Cp) * (U - x(2)) + w(2);
         