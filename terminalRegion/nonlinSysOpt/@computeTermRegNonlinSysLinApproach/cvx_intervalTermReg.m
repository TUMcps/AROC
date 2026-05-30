function IH_semiEdgeLength = cvx_intervalTermReg(var_s,termRegSet)
%CVX_INTERVALTERMREG - parameterized interval enclosure of the 
%       terminal region.
%
% Syntax:
%       res = cvx_intervalTermReg(var_s,termReg)
%
% Description:
%       This function computes the semi edge lengths of the interval
%       enclosure of the parameterized terminal region (parameterized in 
%       the generator scaling factors), see [1, Eq. 25].
%
% Input Arguments:
%       -var_s:         generator scaling factors
%                       (array of dimension [size(termRegSet.G,2),1])
%       -termRegSet:    current candidate for the terminal region
%                       (class: zonotope)
%
% Output Arguments:
%       -IH_semiEdgeLength: semi edge lengths of the interval enclosure of
%                           the paratermized terminal region
%                           (double array of dimension [termReg.dim,1])
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
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

IH_semiEdgeLength = abs(termRegSet.G)*var_s;

end
