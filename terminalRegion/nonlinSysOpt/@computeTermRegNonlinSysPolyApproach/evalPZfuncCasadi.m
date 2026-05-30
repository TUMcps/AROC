function res = evalPZfuncCasadi(pZ,x,id)
%EVALPZFUNCCASADI - adapted from @polyZonotope/resolve (CORA) for compatibility with CasADi
%
% Syntax:
%       res = resolve(pZ,x,id)
%
% Description:
%       This function is adapted from @polyZonotope/resolve (from the CORA 
%       toolbox), which is not compatible with CasADi symbolic variables. 
%       As the original function, this function computes the result of 
%       inserting a value for the identifiers into a polynomial zonotope.
%
% Input Arguments:
%       -pZ:    polyZonotope object to be resolved
%       -x      value to be substituted
%       -id:    ids corresponding to x
%
% Output Arguments:
%       -res:   resulting polynomial zonotope or double if id==pZ.id
%
%
% See Also:
%       @polyZonotope/resolve (CORA toolbox)
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
% Authors:      Victor Gaßmann, Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% check input arguments
if length(id)~=length(x)
    error('Length of id should match length of values!');
end

if sum(ismember(pZ.id,id))~=length(id) || length(unique(id))~=length(id)
    throw(CORAerror('CORA:wrongValue','third','Invalid id'));
end


% sort in ascending order such that ismember later on is enough
[id,indid] = sort(id);
x = x(indid);
[Id,ind] = sort(pZ.id);
eM = pZ.E(ind,:);
% resolve pZ
indx = ismember(Id,id);
eMx = eM(indx,:);
G = pZ.G;
if ~isempty(eMx)
    [n, m] = size(eMx);
    y = ones(1,m);
    for ii = 1:n
        y = y .* (x(ii).^eMx(ii,:));
    end
    G = pZ.G*diag(y);
end
E = eM;
E(indx,:) = [];

% remove zero exponents (logical indexing is not compatibale with casadi
% symbolic variables!)
% ... all-zero columns in exponent matrix
ind = sum(E,1)==0;
idx_rem = find(ind);
idx_keep = find(~ind);
% ... add corresponding columns in dependent generator matrix to the center
c=sum(G(:,idx_rem),2);
% ... remove corresponding columns in dependent generator matrix
G = G(:, idx_keep);
% remove all-zero columns in exponent matrix
E(:,ind) = [];

c = c + pZ.c;
if isempty(E) || all(E(:)==0)
    % nothing left
    c = c + sum(G,2);
    G = [];
    E = [];
    if isempty(pZ.GI) || all(pZ.GI(:)==0)
        res = c;
        return;
    end
end
[E,G] = removeRedundantExponents(E,G);

% revert sort order
ids = pZ.id(~ismember(pZ.id,id));
[~,idx] = sort(ids);
indr_rev(idx) = (1:length(idx));
res = polyZonotope(c, G, pZ.GI, E(indr_rev,:), ids);

end