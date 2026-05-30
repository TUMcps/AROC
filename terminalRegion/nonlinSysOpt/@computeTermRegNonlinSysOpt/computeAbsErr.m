function absErr = computeAbsErr(obj,dynamics,termRegSet)
%COMPUTEABSERR - computes the abstraction error for a given
%       nonlinear dynamical system and a given terminal region.
%
% Syntax:
%       absErr = computeAbsErr(obj,dynamics,termRegSet)
%
% Description:
%       This function computes the abstraction error or a given
%       nonlinear dynamical system over a given terminal region based on
%       linearization- [1] or polymonialization-based [2] reachability
%       algorithms.
%
% Input Arguments:
%       -obj:         instance of class computeTermRegNonlinSysOpt
%       -dynamics:    instance of class termRegNonlinSysDynamics
%       -termRegSet:  candidate for the terminal region 
%                     (class: zonotope/polyZonotope)
%
% Output Arguments:
%       -absErr:    set of abstraction errors
%                   (class: interval)
%
% See Also:
%       -
%
% References:
%       [1] M. Althoff et al., "Reachability Analysis of Nonlinear Systems 
%           with Uncertain Parameters using Conservative Linearization",
%           IEEE Conference on Decision and Control, 2008, pp. 4042-4048.
%       [2] Matthias Althoff, "Reachability analysis of nonlinear systems 
%           using conservative polynomialization and non-convex sets", 
%           International conference on Hybrid systems: computation and 
%           control, 2013, pp. 173–182.
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

linearizationPoint = [obj.xEq;obj.uEq];

% compute interval enclosure of augmented reachable set
% (order reduction to ensure recursive feasibility and simplify
% computation of Lagrange remainder approximations
Zy = reduce(cartProd(termRegSet,obj.Z_U),'girard',1);
Zz = cartProd(Zy,interval(obj.W));

% delta_termSet_y = termSet_y + zonotope(-linearizationPoint);
delta_Zz = Zz + zonotope([-linearizationPoint;zeros(obj.nw,1)]);
dz = max([abs(interval(delta_Zz).inf),abs(interval(delta_Zz).sup)],[],2);

if isa(obj,'computeTermRegNonlinSysPolyApproach')
    % set correct tensor files
    nlSys = setThirdOrderTensor(dynamics.sys,'int');

    % evaluate third-order tensor
    [T,ind] = nlSys.thirdOrderTensor(interval(Zy),interval(obj.W));

    % compute Lagrange remainder
    lagrRem = interval(zeros(nlSys.nrOfDims,1), ...
        zeros(nlSys.nrOfDims,1));
    dz = interval(-dz,dz);
    for i=1:length(ind)
        error_sum = interval(0,0);
        for j=1:length(ind{i})
            error_sum = error_sum + (dz.'*T{i,j}*dz) * dz(j);
        end
        lagrRem(i,1) = 1/6*error_sum;
    end
    % width of interval enclosure
    width = max(abs(lagrRem.sup),abs(lagrRem.inf));

elseif isa(obj,'computeTermRegNonlinSysLinApproach') 
    % bound abstraction error as proposed in [1, Proposition 1]

    % set correct tensor file
    nlSys = setHessian(dynamics.sys,'int');

    % evaluate second-order tensor
    H = nlSys.hessian(interval(Zy),interval(obj.W));

    % compute Lagrange remainder
    width = zeros(length(H),1);
    for ii = 1:length(H)
        H_ = abs(H{ii});
        H_ = max(infimum(H_),supremum(H_));
        width(ii) = 0.5 * dz' * H_ * dz;
    end
end

% assign output argument
absErr = project(interval(-width,width),1:dynamics.nx);

end
