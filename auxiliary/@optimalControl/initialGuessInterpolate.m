function y0 = initialGuessInterpolate(obj,x0,xf)
%INITIALGUESSINTERPOLATE - Initial guess for the optimal control problem.
%
% Syntax:
%       y0 = initialGuessInterpolate(obj,x0,xf)
%
% Description:
%       This function computes an initial guess for the optimal control
%       problem by linearly interpolating between the initial and a given
%       final state. The control inputs as well as all auxiliary varibales
%       (e.g. slack variables) are set to zero.
%
% Input Arguments:
%       -obj:       instance of class optimalControl
%       -x0:        initial state
%                   (double array of dimension [obj.nx,1])
%       -xf:        final state
%                   (double array of dimension [obj.nx,1])
%
% Output Arguments:
%       -y0:        initial guess for the optimal control problem
%                   (double array of dimension [obj.idxVars.nVars,1])
%
% See Also:
%       -
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
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

y0 = zeros(obj.idxVars.nVars,1);

alpha = linspace(1, 0, obj.N+1);
y0(obj.idxVars.x) = alpha.*x0 + (1-alpha).*xf;

end
