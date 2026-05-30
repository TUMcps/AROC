function [cIneqStruct, cIneqStructAux] = approxAbsErrPwLin_con(obj,candTermReg,paramAbsErr_coefficients,idxVars)
%APPROXABSERRPWLIN_CON - returns the (linear inequality) constraints 
%       encoding the piece-wise linear approximation of the abstraction error.
%
% Syntax:
%       [cIneqStruct, cIneqStructAux] = approxAbsErrPwLin_con(obj,candTermReg,paramAbsErr_coefficients,idxVars)
%
% Description:
%       This function returns the  (linear inequality) constraints 
%       encoding the piece-wise linear approximation of the abstraction 
%       error proposed in [1, Example 1].
%
% Input Arguments:
%       -obj:                       instance of class
%                                   computeTermRegNonlinSysPolyApproach
%       -candTermReg:               current candidate for the terminal 
%                                   region
%                                   (polyZonotope/struct storing the 
%                                   parameters of a polyZonotope)
%       -paramAbsErr_coefficients:  struct storing the coefficients of 
%                                   the piece-wise linear approximation of 
%                                   the abstraction error
%               .origin:    upper bound on the abstraction error at 0xUxW
%                           (array of dimension [obj.nx,1])
%               .termReg:   upper bound on the abstraction error at 
%                           box(candTermReg)xUxW
%                           (array of dimension [obj.nx,1])
%               .tr         upper bound on the abstraction error at 
%                           box(candTermReg+trust_region)xUxW
%                           (array of dimension [obj.nx,1])
%       -idxVars:                   struct storing the indices of the 
%                                   optimization variables
%
% Output Arguments:
%       -cIneqStruct:       struct storing the matrix and right-hand
%                           side of the linear inequality constraint 
%                           ensuring [1, Eq. (20b)]
%       -cIneqStructAux:    struct storing the matrix and right-hand
%                           side of the auxiliary linear inequality 
%                           constraints
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/approxAbsErrPwLin_coefficients
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
% Copyright (c) 2026 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

% evaluation constraints -------------------------------------------------
% abstraction error must be greater or equal than lower (piece-wise linear)
% bound

count_constr = 0;
Aineq.col = [];
Aineq.row = [];
Aineq.val = [];
bineq.row = [];
bineq.val = [];

% 1st lower bound: origin - candidate terminal region
tmp_coeff = paramAbsErr_coefficients.termReg - paramAbsErr_coefficients.origin;
Aineq.row = [Aineq.row, repmat(count_constr+(1:idxVars.n_sAbsErr),1,2)];
Aineq.col = [Aineq.col, ...
    idxVars.sAbsErr,repelem(idxVars.auxSAbsErr(1),1,idxVars.n_sAbsErr)];
Aineq.val = [Aineq.val, -paramAbsErr_coefficients.termReg', tmp_coeff'];
% Aineq.val = [Aineq.val, -ones(1,idxVars.n_sAbsErr), tmp_coeff'];
bineq.row = [bineq.row, count_constr+(1:idxVars.n_sAbsErr)];
bineq.val = [bineq.val, (-paramAbsErr_coefficients.origin)'];

count_constr = count_constr + idxVars.n_sAbsErr;

% 2nd lower bound: candidate terminal region - trust region
tmp_coeff = paramAbsErr_coefficients.tr - paramAbsErr_coefficients.termReg;
Aineq.row = [Aineq.row, repmat(count_constr+(1:idxVars.n_sAbsErr),1,2)];
Aineq.col = [Aineq.col, ...
    idxVars.sAbsErr,repelem(idxVars.auxSAbsErr(2),1,idxVars.n_sAbsErr)];
Aineq.val = [Aineq.val, -paramAbsErr_coefficients.termReg', tmp_coeff'];
% Aineq.val = [Aineq.val, -ones(1,idxVars.n_sAbsErr), tmp_coeff'];
bineq.row = [bineq.row, count_constr+(1:idxVars.n_sAbsErr)];
bineq.val = [bineq.val, (-paramAbsErr_coefficients.termReg)'];

count_constr = count_constr + idxVars.n_sAbsErr;

% set constraint ingredients
tmp_Aineq = sparse(Aineq.row,Aineq.col,Aineq.val,count_constr,idxVars.nVars);
tmp_bineq = sparse(bineq.row, ones(1,length(bineq.row)), bineq.val,count_constr,1);
nnzRows = find(~sum(tmp_Aineq~=0,2) == 0);
cIneqStruct.Aineq = tmp_Aineq(nnzRows,:);
cIneqStruct.bineq = tmp_bineq(nnzRows,1);
cIneqStruct.nCon = length(nnzRows);

% auxiliary constraints --------------------------------------------------

count_constr = 0;
Aineq.col = [];
Aineq.row = [];
Aineq.val = [];
bineq.row = [];
bineq.val = [];

% uniform scaled interval enclosure of initial candidate terminal region
% contains terminal region (1st variable in auxSAbsErr)
% ... interval enclosure of candidate terminal region 
IH0 = interval(zonotope(candTermReg.c,candTermReg.G) - obj.xEq);
dx0 = max(abs(infimum(IH0)),abs(supremum(IH0)));

% ... compute arguments of parameterized approximation
n_dim = length(obj.argAbsErr);
% ... positive direction, i.e., +e_j
% -> generators
Aineq.row = [Aineq.row, repmat(count_constr+(1:n_dim),1,idxVars.n_s)];
Aineq.col = [Aineq.col,repelem(idxVars.s,1,n_dim)];
tmp_val = abs(candTermReg.G(obj.argAbsErr,:));
Aineq.val = [Aineq.val, tmp_val(:)'];
% -> variable part of center
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, idxVars.delta_c(obj.argAbsErr)];
Aineq.val = [Aineq.val, ones(1,n_dim)];
% -> aux. variable (scaling factor)
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, repelem(idxVars.auxSAbsErr(1),1,n_dim)];
Aineq.val = [Aineq.val, -dx0(obj.argAbsErr,1)']; 

count_constr = count_constr + n_dim;

% ... negative direction, i.e., -e_j
% -> generators
Aineq.row = [Aineq.row, repmat(count_constr+(1:n_dim),1,idxVars.n_s)];
Aineq.col = [Aineq.col,repelem(idxVars.s,1,n_dim)];
tmp_val = abs(candTermReg.G(obj.argAbsErr,:));
Aineq.val = [Aineq.val, tmp_val(:)'];
% -> variable part of center
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, idxVars.delta_c(obj.argAbsErr)];
Aineq.val = [Aineq.val, -ones(1,n_dim)];
% -> aux. variable (scaling factor)
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, repelem(idxVars.auxSAbsErr(1),1,n_dim)];
Aineq.val = [Aineq.val, -dx0(obj.argAbsErr,1)'];

count_constr = count_constr + n_dim;

% exploitation of trust region (second variable in auxSAbsErr)
% assume trust region is centered at the origin
dx_tr = dx0 + interval(obj.tr_set).sup;
% ... positive direction, i.e., +e_j
% -> generators
Aineq.row = [Aineq.row, repmat(count_constr+(1:n_dim),1,idxVars.n_s)];
Aineq.col = [Aineq.col,repelem(idxVars.s,1,n_dim)];
tmp_val = abs(candTermReg.G(obj.argAbsErr,:));
Aineq.val = [Aineq.val, tmp_val(:)'];
% -> variable part of center
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, idxVars.delta_c(obj.argAbsErr)];
Aineq.val = [Aineq.val, ones(1,n_dim)];
% -> aux. variable (scaling factor)
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, repelem(idxVars.auxSAbsErr(2),1,n_dim)];
Aineq.val = [Aineq.val, ...
    -(dx_tr(obj.argAbsErr) - dx0(obj.argAbsErr))']; 
% right-hand side
bineq.row = [bineq.row, count_constr+(1:n_dim)];
bineq.val = [bineq.val, dx0(obj.argAbsErr)'];

count_constr = count_constr + n_dim;

% ... negative direction, i.e., -e_j
% -> generators
Aineq.row = [Aineq.row, repmat(count_constr+(1:n_dim),1,idxVars.n_s)];
Aineq.col = [Aineq.col,repelem(idxVars.s,1,n_dim)];
tmp_val = abs(candTermReg.G(obj.argAbsErr,:));
Aineq.val = [Aineq.val, tmp_val(:)'];
% -> variable part of center
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, idxVars.delta_c(obj.argAbsErr)];
Aineq.val = [Aineq.val, -ones(1,n_dim)];
% -> aux. variable (scaling factor)
Aineq.row = [Aineq.row, count_constr+(1:n_dim)];
Aineq.col = [Aineq.col, repelem(idxVars.auxSAbsErr(2),1,n_dim)];
Aineq.val = [Aineq.val, ...
    -(dx_tr(obj.argAbsErr) - dx0(obj.argAbsErr))']; 
% right-hand side
bineq.row = [bineq.row, count_constr+(1:n_dim)];
bineq.val = [bineq.val, dx0(obj.argAbsErr)'];

count_constr = count_constr + n_dim;

% trust-region-like constraint: terminal region contained in trust region
% <=> exploitation of trust region <= 1
Aineq.row = [Aineq.row, count_constr+1];
Aineq.col = [Aineq.col, idxVars.auxSAbsErr(2)];
Aineq.val = [Aineq.val, 1];
bineq.row = [bineq.row, count_constr+1];
bineq.val = [bineq.val, 1];

count_constr = count_constr + 1;

% set constraint ingredients
cIneqStructAux.Aineq = sparse(Aineq.row,Aineq.col,Aineq.val,count_constr,idxVars.nVars);
cIneqStructAux.bineq = sparse(bineq.row, ones(1,length(bineq.row)), bineq.val,count_constr,1);
cIneqStructAux.nCon = count_constr;

end
