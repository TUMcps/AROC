function [quadCoeff,bilinCoeff,trCoeff] = approxAbsErrQuad_coefficients(obj,nlSys,termRegSet,W,varargin)
%APPROXABSERRQUAD_COEFFICIENTS - coefficients of the quadratic 
%       approximation of the abstraction error.
%
% Syntax:
%       [quadCoeff,bilinCoeff,trCoeff] = approxAbsErrQuad_coefficients(obj,nlSys,termRegSet,W)
%
% Description:
%       This function computes the coefficients of the quadratic
%       approximation of the abstraction error proposed in [1, Proposition
%       1].
%
% Input Arguments:
%       -obj:           instance of class 
%                       computeTermRegNonlinSysLinApproach         
%       -nlSys:         nonlinear system object
%                       (class: nonlinearSys / nonlinearSysDT)
%       -termRegSet:    candidate terminal region
%                       (class: zonotope)
%       -W:             set of disturbances
%                       (class: zonotope)
%   
% Output Arguments:
%       -quadCoeff:     coefficients of the quadratic terms [1, Eq. 27a]
%                       (array of dimension [obj.nx,1])
%       -bilinCoeff:    coefficients of the bilinear terms in [1, Eq. 27a]
%                       (array of dimension [obj.nx,1])
%       -trCoeff:       coefficients of the linear term, see [1, Eq. 27b]
%                       (array of dimension [obj.nx,1])
%
% See Also:
%       @computeTermRegNonlinSysLinApproach/approxAbsErrQuad_eval
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

if isa(nlSys,'nonlinParamSys')
    paramInt = varargin{1};
end

linearizationPoint = [obj.xEq;obj.uEq];
termRegSet_y = cartProd(termRegSet,obj.Z_U);
termRegSet_tr = termRegSet + obj.tr_set;
termRegSet_tr_y = cartProd(termRegSet_tr,obj.Z_U);
IHw = interval(W);
dw = max(abs(IHw.inf),abs(IHw.sup));
IHy = interval(termRegSet_y) + (-linearizationPoint);
dz = [max(abs(IHy.sup),abs(IHy.inf));dw];
IHy_tr = interval(termRegSet_tr_y + (-linearizationPoint));
dz_tr = [max(abs(IHy_tr.inf),abs(IHy_tr.sup));dw];

% evaluate the hessians 
nlSys = setHessian(nlSys,'int');
if isa(nlSys,'nonlinParamSys')
    H = nlSys.hessian(interval(IHy + linearizationPoint),IHw,paramInt);
else
    H = nlSys.hessian(interval(IHy + linearizationPoint),IHw);
end
for ii = 1:obj.nx
    H_ = abs(H{ii});
    H{ii} = max(infimum(H_),supremum(H_));
end
if isa(nlSys,'nonlinParamSys')
    H_tr = nlSys.hessian(interval(IHy_tr + linearizationPoint),IHw,paramInt);
else
    H_tr = nlSys.hessian(interval(IHy_tr + linearizationPoint),IHw);
end
for ii = 1:obj.nx
    H_ = abs(H_tr{ii});
    H_tr{ii} = max(infimum(H_),supremum(H_));
end

% get coefficients of quadratic terms -> extract diagonal entries
quadCoeff = zeros(obj.nx,obj.nx+obj.nu+obj.nw);
for ii = 1:obj.nx
    quadCoeff(ii,:)= reshape(0.5*spdiags(H{ii},0),[1,obj.nx+obj.nu+obj.nw]);
end

% compute coefficients of bilinear terms -> eavaluate off-diagonal elements 
% at current candidate terminal region
bilinCoeff = zeros(obj.nx,1);
for ii = 1:obj.nx
    bilinCoeff(ii) = 0.5*dz'*H{ii}*dz - quadCoeff(ii,:)*dz.^2;
end

% compute coefficients of trust-region terms
trCoeff = zeros(obj.nx,1);
for ii = 1:obj.nx
    trCoeff(ii) = 0.5*dz_tr'*H_tr{ii}*dz_tr - quadCoeff(ii,:)*(dz_tr.^2) ...
        -bilinCoeff(ii);
end

end
