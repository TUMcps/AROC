function [pval,zval] = strictFeasZ(p,A_pz,b_pz,TOL)
% STRICTFEASZ - find feasible values for linear inequality constraint
%
% Syntax:
%       [pval,zval] = STRICTFEASZ(p,A_pz,b_pz,TOL)
%
% Description:
%       Find feasible values for the variables p and z such that the
%       inequality A_pz * [p;z] <= b_pz is satisfied.
%
% Input Arguments:
%
%       -p:     initial value for the variable p
%       -A_pz:  matrix for the inequality constraint
%       -b_pz:  vecotr for the inequality constraint
%       -TOL:   tolerance
%
% Output Arguments:
%
%       -pval:  feasible value for the variable p
%       -zval:  feasible value for the variable z
%
% See Also:
%       polynomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% A_pz*[p;z]<=b_pz
[n_s,n_pz] = size(A_pz);
n_p = length(p);
n_z = n_pz-n_p;

% try to find z such that b_pz-A_pz*[p;z] -> max (for strict feasibility)
ii_z = find(A_pz(:,end),1);
n_half = round((n_s-ii_z)/2);
A_p_pos = A_pz(ii_z+1:ii_z+n_half,1:n_p);
b_pos = b_pz(ii_z+1:ii_z+n_half);
A_zsum = A_pz(1:ii_z,n_p+1:end);
A_z = A_pz(ii_z+1:ii_z+n_half,n_p+1:end);

% find independent p, z
z = zeros(n_z,1);

for i = 1:ii_z
    ind_zi = A_zsum(i,:)~=0;
    nz_i = sum(ind_zi);
    A_pi = A_p_pos(ind_zi,:);
    in_abs_i = b_pz(i)-A_zsum(i,ind_zi)*abs(A_pi*p-b_pos(ind_zi));
    chunk_i = in_abs_i/(1+nz_i);
    z(ind_zi) = abs(A_pi*p-b_pos(ind_zi)) + chunk_i;
end

% now check if all tolerances are met
if all(b_pz-A_pz*[p;z]>=TOL-1e-6)
    pval = p;
    zval = z;
    return;
end

% if infeasible, find p that respects tolerance and is feasible
options = optimoptions(@quadprog,'display','none');

% linear inequalities
A = [A_pz,eye(n_s)];
b = b_pz;

% bounds
lb = [-ones(n_p,1);zeros(n_z,1);TOL*ones(n_s,1)];
ub = [ones(n_p,1);ones(n_z,1);inf(n_s,1)];
H = blkdiag(eye(n_p),zeros(n_z+n_s));
f = [-p;zeros(n_z+n_s,1)];
[xval,~,exitflag] = quadprog(H,f,A,b,[],[],lb,ub,[],options);
assert(exitflag>=0,'Fix!');
pval = xval(1:n_p);
zval = xval(n_p+1:n_p+n_z);