function partZono = paramPartZonotope(pZ,idv,idp)
%PARAMPARTZONOTOPE - wrapper for computing a partial zonotope
%       over-approximation of the reachable set.
%
% Syntax:
%      partZono = computeTermRegNonlinSysPolyApproach.paramPartZonotope(pZ,idv,idp) 
%
% Description:
%       This function computes a partial zonotope over-approximation of a
%       given polynomial zonotope pZ in the ids stored in idv (which 
%       include the generator scaling factors). Since we scale the 
%       generators of the initial set during optimization, we cannot use 
%       the partZonotope method of the polyZonotope class, which ignores 
%       the dependance of the center of pZ on the scaling factors.
%
% Input Arguments:
%       -pZ:        parameterized reachable set
%                   (class: polyZonotope)
%       -idv:       vector of ids for over-approximation (interval-valued
%                   factors like generator scaling factors)
%       -idp:       vector of ids of scalar-valued factors like the
%                   controller parameters
%
% Output Arguments:
%       partZono:   struct storing the components of the partial
%                   zonotope over-approximation of pZ:
%                   .c: center - does not include contributions by
%                       generators with exlusively even exponents in idv
%                       (class: polyZonotope)
%                   .G: generators
%                       (class: cell array of polyZonotopes)
%                   .E: exponent matrix of factors with id in idv 
%                       (integer array of dimension [length(idv,
%                       length(partZono.G)])
%                   .isConstGen: logical vector indicating whether
%                       partZono{i}.G is indepdendent (true) of ids in idp 
%                       or not (false)
%                       (logical vector of length length(partZono.G))
%                   .cVar: logical vector indicating whether partZono{i}.G
%                       has exclusively even exponents in idv (true -
%                       contributes to the center of the re-assembled
%                       polyZonotope) or not (false)
%                       (logical vector of length length(partZono.G))
%                   .idv: ids of interval-valued factors like generator 
%                       scaling factors)
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control Invariant
%           Sets of Nonlinear Systems Using Polynomial Controller
%           Synthesis," American Control Conference, 2024, pp. 4162-4169.
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

% restore Ids
pZ = restoreId(pZ,idv);
pZ = restoreId(pZ,idp);
[~,idx_idv] = intersect(pZ.id,idv,'stable');
[~,idx_idp] = intersect(pZ.id,idp,'stable');

if ~isempty(setdiff(pZ.id,vertcat(idv,idp)))
    error(['Error in preparing partial zonotope over-approximation. ', ...
        'pZ contains ids neither in idv nor in idp!'])
end

% partial zonotope over-approximation
[~,pZ_gi,isConstGen,expMat_gi,~] = partZonotope(pZ,idv);

% post-process result ----------------------------------------------------

% re-scale elements of pZ_gi with exclusively even exponents (in idv)
cVar = false([1,length(pZ_gi)-1]);
for ii = 2:length(pZ_gi)
    if all(mod(expMat_gi(:,ii),2)==0)
        pZ_gi{ii} = pZ_gi{ii}*2;
        cVar(ii-1) = true;
    end
end

% extract constant in idv part of center of partial zonotope over-approx.
cols = find(sum(pZ.E(idx_idv,:),1) == 0);
pZ_gi{1} = polyZonotope(pZ.c,pZ.G(:,cols),[], ...
    pZ.E(idx_idp,cols),pZ.id(idx_idp));

% assign output values ---------------------------------------------------
partZono.idv = idv;
partZono.c = pZ_gi{1};
partZono.G = reshape(pZ_gi(2:end),1,length(pZ_gi)-1);
partZono.GI = pZ.GI;
partZono.E = expMat_gi(:,2:end);
partZono.isConstGen = isConstGen(2:end);
partZono.cVar = cVar;

end
