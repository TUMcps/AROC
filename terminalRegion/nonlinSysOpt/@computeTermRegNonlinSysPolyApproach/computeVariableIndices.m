function idxVars = computeVariableIndices(obj,id,id_redVar,nGenR,nGenTermReg,nGenCtrl,nGenU,nGenReduce,nGenErr)
% COMPUTEVARIABLEINDICES - computes the indices of the entries storing, 
%       e.g., the generator scaling factors in the vector of optimization 
%       variables.
%
% Syntax:
%       idxVars = computeVariableIndices(obj,id,id_redVar,nGenR,nGenTermReg,nGenCtrl,nGenU,nGenReduce,nGenErr)
% 
% Description:
%       To formulate the polynomial program in [1, Eq. (19)] in the generic 
%       form of a nonlinear programming problem, i.e., 
%           min f(y) s.t. g(y) <= 0, h(y) = 0,
%       the variables must be lumped together into a single vector y. To, 
%       e.g., easily extract the generator scaling factors from the optimal 
%       solution, this function computes the indices of the entries of y 
%       storing the corresponding variable.
%       These indices are stored in the struct idxVars. They need to be
%       recomputed in every iteration since the number of reduced
%       generators and, thus, the number of variables might change in
%       between iterations.
%
% Input Arguments:
%       -obj:           instance of class
%                       computeTermRegNonlinSysPolyApproach       
%       -id:            struct storing the polynomial zonotope ids for
%                       parameterized reachability analysis 
%       -id_redVar:     set of indices indicating reducing generators with
%                       variable length (in the optimization variables)
%       -nGenR:         number of dependent generators of the parameterized 
%                       reachable set
%                       (scalar)
%       -nGenTermReg:   number of generators of the terminal region
%                       (scalar)
%       -nGenCtrl:      number of generators of the controller template
%                       (scalar)
%       -nGenU:         number of generators of the set of admissible
%                       control inputs
%                       (scalar)
%       -nGenReduce:    number of reduced generators (parameterized order
%                       reduction)
%                       (scalar)
%       -nGenErr:       number of independent generators of the
%                       parameterized reachable set
%                       (scalar)
%
% Output Arguments:
%       -idxVars:       struct storing the indices of the optimization
%                       variables
%
% See Also:
%       @computeTermRegNonlinSysPolyApproach/optimizeRCIpoly, 
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

% variables representing scalar ids --------------------------------------
% assumption: id.var_scalar are -1, -2, ...
% -> make variable indices independent of order of ids

% ... controller parameters
idxVars.p = reshape(abs(obj.id.p),1,[]); 
idxVars.n_p = obj.ctrl.np;

% ... center translation
idxVars.delta_c = reshape(abs(obj.id.delta_c ),1,[]); 
idxVars.n_dc = obj.nx;

counter = idxVars.n_p + idxVars.n_dc;

% variables representing scaling ids -------------------------------------
% assumption: id.var_scaling (termReg, red, absErr) are 1, 2, ...
% -> make variable indices independent of order of ids

% ... terminal region generator scaling factors
idxVars.s = counter + reshape(obj.id.termReg,1,[]);
idxVars.n_s = length(idxVars.s);

% scaling factors of reducing generators
% ... find scaling factors that are constant (length of the corresponding 
% generators is guaranteed to be zero) 
id_redVar = reshape(id_redVar,1,[]);
idx_const = find(~ismember(id.red,id_redVar));
idx_nonConst = find(ismember(id.red,id_redVar));
% ... split ids
id_redConst = obj.id.red(idx_const);
id_redNonConst = obj.id.red(idx_nonConst);
% ... set constant variables
idxVars.sRedConst = counter + reshape(id_redConst,1,[]);
idxVars.n_sRedConst = length(idxVars.sRedConst);
% ... set non-constant variables
idxVars.sRed = counter + reshape(id_redNonConst,1,[]);
idxVars.n_sRed = length(idxVars.sRed);

% scaling factors of abstraction error
idxVars.sAbsErr = counter + reshape(obj.id.absErr,1,[]);
idxVars.n_sAbsErr = length(idxVars.sAbsErr);

counter = counter + idxVars.n_s + idxVars.n_sRed + idxVars.n_sAbsErr;

% collect variables involved in nonlinear constraints --------------------
% generators of reachable set, etc.
idxVars.z = sort(horzcat(idxVars.p, idxVars.delta_c, ...
        idxVars.s, idxVars.sRed, idxVars.sRedConst, idxVars.sAbsErr));
idxVars.n_z = length(idxVars.z);

% auxiliary variables ----------------------------------------------------

% auxiliary variables to compute lower bounds of approximation of
% abstraction error
idxVars.auxSAbsErr = counter + (1:length(id.absErr_aux));
idxVars.n_auxSAbsErr = length(idxVars.auxSAbsErr);
counter = counter + 2;

% scaling factors of "reduced" generators - auxiliary variables
idxVars.auxSRed = counter + (1:idxVars.n_sRed*nGenReduce);
idxVars.n_auxSRed = length(idxVars.auxSRed);
counter = counter + idxVars.n_auxSRed;

% invariance constraint
idxVars.n_GammaInv = nGenTermReg*(nGenR+nGenErr);
idxVars.GammaInv = counter+1:counter+idxVars.n_GammaInv;
counter = counter+idxVars.n_GammaInv;
idxVars.n_gammaInv = nGenTermReg;
idxVars.gammaInv = counter+1:counter+idxVars.n_gammaInv;
counter = counter+idxVars.n_gammaInv;

% invariance constraint - auxiliary variables
idxVars.n_auxGammaInv = idxVars.n_GammaInv;
idxVars.auxGammaInv = counter+1:counter+idxVars.n_auxGammaInv;
counter = counter+idxVars.n_auxGammaInv;
idxVars.n_auxBetaInv = idxVars.n_gammaInv;
idxVars.auxBetaInv = counter+1:counter+idxVars.n_auxBetaInv;
counter = counter+idxVars.n_auxBetaInv;

% invariance constraint - slack variables
idxVars.n_slack = 1;
idxVars.slack = counter+1:counter+idxVars.n_slack;
counter = counter+idxVars.n_slack;

% input constraint
idxVars.n_GammaU = nGenU*nGenCtrl;
idxVars.GammaU = counter+1:counter+idxVars.n_GammaU;
counter = counter+idxVars.n_GammaU;
idxVars.n_gammaU = nGenU;
idxVars.gammaU = counter+1:counter+idxVars.n_gammaU;
counter = counter+idxVars.n_gammaU;

% input constraint - auxiliary variables 
idxVars.n_auxGammaU =idxVars.n_GammaU;
idxVars.auxGammaU = counter+1:counter+idxVars.n_auxGammaU;
counter = counter+idxVars.n_auxGammaU;
idxVars.n_auxBetaU = idxVars.n_gammaU;
idxVars.auxBetaU = counter+1:counter+idxVars.n_auxBetaU;
counter = counter+idxVars.n_auxBetaU;

if ~obj.flagCostExact
    % scaling factors of cost set
    idxVars.sCost = counter + (1:length(id.cost));
    idxVars.n_sCost = length(idxVars.sCost);
    counter = counter + idxVars.n_sCost;

    % reduced cost set containment constraint
    idxVars.n_GammaCost = idxVars.n_s*idxVars.n_sCost;
    idxVars.GammaCost = counter + (1:idxVars.n_GammaCost);
    counter = counter + idxVars.n_GammaCost;
    
    % reduced cost set contaiment constraint - auxiliary variables
    idxVars.n_auxGammaCost = idxVars.n_GammaCost;
    idxVars.auxGammaCost = counter + (1:idxVars.n_auxGammaCost);
    counter = counter + idxVars.n_auxGammaCost;
else
    idxVars.sCost = idxVars.s;
    idxVars.n_sCost = idxVars.n_s;
end

idxVars.nVars = counter;

end
