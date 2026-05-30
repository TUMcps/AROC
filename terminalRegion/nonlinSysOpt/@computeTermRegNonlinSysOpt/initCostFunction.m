function obj = initCostFunction(obj,initialGuessSet)
%INITCOSTFUNCTION - computes the coefficients of the cost function using
%       the intial guess for the terminal region. 
%
% Syntax:
%       obj = initCostFunction(obj,initialGuessSet)
%
% Description:
%       This function computes the coefficients of the cost function 
%       (approximation of the volume of the terminal region) using the 
%       intial guess for the terminal region. If the number of coefficients
%       exceeds a given threhold, a set of reduced order that is contained 
%       in the terminal region is computed, which will be used to obtain an 
%       underapproximation of the cost function value (see [1, Remark 2]).
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -initialGuessSet:   initial guess for the terminal region
%                           (class: zonotope)
%
% Output Arguments:
%       -obj:               updated instance of class
%                           computeTermRegNonlinSysOpt
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", 2024, IEEE Transactions 
%           on Automatic Control
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
    
if ~obj.flagCostExact
    % compute set of order obj.costCostSetOrder that is inscribed into the
    % initial guess for the terminal region
    termReg_red = reduce(zonotope(initialGuessSet),'pca',obj.costSetOrder);        
    costExponents = combinator(size(termReg_red.G,2),obj.nx,'c');
    
    % ... uniform scaling to ensure containment
    scale = scaleZonoIntoZono(termReg_red,zonotope(initialGuessSet));
    if isa(obj,'computeTermRegNonlinSysPolyApproach')
        termReg_red = polyZonotope(termReg_red.c, termReg_red.G*diag(scale), ...
            [],eye(length(obj.id.cost)),obj.id.cost);
    else
        termReg_red = zonotope(termReg_red.c, termReg_red.G*diag(scale));
    end
else
    termReg_red = initialGuessSet;
    costExponents = combinator(size(termReg_red.G,2),obj.nx,'c');

end

% evaluate coefficients
costCoefficients = zeros(size(costExponents,1),1);
for ii = 1:size(costCoefficients,1)
    costCoefficients(ii) = abs(det(termReg_red.G(:,costExponents(ii,:))));
end

% set cost function-related properties
obj.costCoefficients = costCoefficients;
obj.costExponents = costExponents;
obj.costSet = termReg_red;

end
