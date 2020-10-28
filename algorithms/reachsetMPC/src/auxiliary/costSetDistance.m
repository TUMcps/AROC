function Jset = costSetDistance(R,Opts)
% COSTSETDISTANCE - sum of distances of the reachable sets to the terminal 
%                   region
%
% Syntax:
%       Jset = COSTSETDISTANCE(R,poly,alpha)
%
% Description:
%       This function computes the sum of distances of reachable sets at
%       discrete time points to the terminal region (see Eq. (13) in [1]).
%
% Input Arguments:  
%
%       -R:     Cell array containing the reachable sets at discrete time
%               points
%       -Opts:  a structure containing following options
%
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope)
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%
% Output Arguments:
%
%       -Jset:  Summed distances of the points in x to the terminal region
%
% See Also:
%       reachsetMPC
%
% References:
%       * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
%              for Disturbed Nonlinear Systems
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


    % get polytope parameter
    A = Opts.termReg.A;
    b = Opts.termReg.b;
    b = b*1/(1+Opts.alpha);
    
    % compute costs for the reachable sets
    Jset = 0;
    for i = 1:length(R)
        Jset = Jset + distPhi(A,b,R{i});
    end
end