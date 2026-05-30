function [R_next,mapsParamReduce] = reduceTemplate(obj,R_abs,R_err,candTermReg,candP,flagReduceFeasible)
%REDUCETEMPLATE - parameterized order reduction of the parameterized 
%       reachable set
%
% Syntax:
%       [R_next,mapsParamReduce] = reduceTemplate(obj,R_abs,R_err,candTermReg,candP,flagReduceFeasible)
%
% Description:
%       To limit the number of optimization variables and constraints, the 
%       order of the parameterized reachable set has to be reduced. 
%       However, standard techniques for order reduction of (polynomial)
%       zonotopes would drop the dependence of the reduced generators on
%       the optimization variables. This function returns the reduced
%       reachable set as well as maps for computing the edge lengths of the
%       reducing set (box enclosing the reduced generators) as functions of 
%       the optimization variables. The reduced generators are enclosed by 
%       a box as in [2, Sec. 4]. If we want to ensure recursive feasibility 
%       (see [1, Sec. III-D], the number of reduced generators is chosen 
%       automatically so that the reduced reachable set R_next is contained 
%       in the candidate terminal region.
%
% Input Arguments:
%       -obj:                   instance of the class 
%                               computeTermRegNonlinSysPolyApproach
%       -R_abs:                 parameterized reachable set of the
%                               abstraction
%                               (class: polyZonotope)
%       -R_err:                 approximation of the reachable set of
%                               abstraction errors
%                               (class: polyZonotope)
%       -candTermReg:           candidate terminal region
%                               (class: polyZonotope)
%       -candP:                 candidate controller parameters
%                               (double vector)
%       -flagReduceFeasible:    boolean indicating whether recursive
%                               feasiblity shall be enforced for the 
%                               reduced reachable set or not
%                               (logical)
%
% Output Arguments:
%       - R_next:           reduced approximation of the reachable set
%                           (class: polyZonotope)
%       - mapsParamReduce:  struct storing the maps for computing the
%                           lengths of the reducing set's generators
%                       -.dim:          dimension (=number of parameterized
%                                       reducing generators)
%                       -.c:            dummy value to setup the function
%                                       handles for optimization
%                       -.G:            (polynomial) maps for computing the 
%                                       scaling factors of the reducing
%                                       generators
%                                       (cell array of polyZonotopes)
%                       -.E:            exponent matrix (ids in
%                                       obj.id.termReg)
%                       -.idv:          ids coresponding to .E
%                       -.isConstGen:   ith element is true if .G{i} is
%                                       independent of obj.id.var_scalar, 
%                                       false otherwise
%                       -.id:           ids of reducing generators with
%                                       variable length
%                       -.initScales:   current length of the reducing
%                                       generators
%
% See Also:
%       -
% 
% References:
%       [1] L. Schäfer and M. Althoff, "Computing Robust Control 
%           Invariant Sets of Nonlinear Systems Using Polynomial 
%           Controller Synthesis," American Control Conference, 2024, 
%           pp. 4162-4169. 
%       [2] A. Girard. "Reachability of uncertain linear systems using
%           zonotopes", Hybrid Systems: Computation and Control, 2005, 
%           pp. 291–305.
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

% initial guess for terminal region has no center translation
delta_c = zeros(obj.nx,1);
candTermReg = resolve(candTermReg,delta_c,obj.id.delta_c);

% augment initial guess for controller paramters with initial guess for center
candPC = [candP; delta_c];

% partial zonotope over-approximation of reachable set of abstraction
[R_abs_po] = obj.paramPartZonotope( ...
    R_abs,obj.id.termReg,obj.id.var_scalar);

% select generators for reduction
if flagReduceFeasible
    idx_red = selectGeneratorsRecFeas(R_abs_po,R_err, ...
        candTermReg,candPC,obj.paramReachSetOrder,obj.nx,obj.id.var_scalar);
else
    idx_red = selectGenerators(R_abs_po,candPC, ...
        obj.paramReachSetOrder,obj.nx,obj.id.var_scalar);
end

% re-assemble reachable set of abstraction after reduction
% (GI is dropped intentionally but considered in the next step)
R_abs_red = reassembleDepPart(R_abs_po,obj.id.termReg,idx_red,obj.nx);

% enclose reduced generators by the template and compute parameterized maps
% ... template: box
template = zonotope(zeros(obj.nx,1), eye(obj.nx));
[R_red, mapsParamReduce] = computeParameterizedMaps(obj, ...
    R_abs_po,R_err,idx_red,candPC,template);

% compute reduced time-point reachable set
R_next = exactPlus(R_abs_red,R_red);

end


%% auxiliary functions ---------------------------------------------------

function idx_red = selectGenerators(R_abs_po,candPC,order,nx,id_pc)
% This function selects the generators of the reachable set of the 
% abstraction for reduction (evaluated for the current candidate controller 
% parameters). The order of the resulting zonotope over-approximation is
% fixed to the input argument order. 
%
% Additional References:
%       [3] N. Kochdumper et al. 'Sparse Polynomial Zonotopes: A Novel Set 
%           Representation for Reachability Analysis', 2021, IEEE
%           Transactions on Automatic Control
% 
% ------------------------------------------------------------------------

% desired order only concerns projection onto state space
nGenMax = floor(nx*order);
nRed = length(R_abs_po.G) - nGenMax;
if nRed > 0
    % number of generators to be kept
    nKeep = nGenMax-nx;  
   
    % evaluate generators for current value of p,c
    tmp = cellfun(@(x) resolve(x,candPC,id_pc),R_abs_po.G, ...
        'UniformOutput',false);
    G = cell2mat(reshape(tmp,1,length(R_abs_po.G)));
    G = G(1:nx,:);

    % compute metric for selection of generators based on [3]
    m_dep = sum(abs(G),1);

    % select the smallest nRed generators (= generators to be removed)
    [~,tmp_idx] = sort(m_dep,'descend');
    idx_red = tmp_idx(nKeep+1:end);
else
    idx_red = [];
end

end


function idx_red = selectGeneratorsRecFeas(R_abs_po,R_err,candTermReg, ...
    candPC,order,nx,id_pc)
% This function selects the generators of the reachable set of the 
% abstraction for reduction (evaluated for the current candidate controller 
% parameters). To ensure recursive feasibility (the reduced reachable set
% must be contained in the candidate terminal region), the order is chosen
% automatically to satisfy this condition. An initial guess for the
% zonotope order is provided via the input argument 'order'.
%
% ------------------------------------------------------------------------

    function tmp_Rred = computeReducedReachSet(c_red, G_abs, expMat_abs, ...
            R_err_G, R_err_GI,tmp_idx,nGenAbs) 
        
        % compute reduced reachable set - reduction
        G_red = diag(sum(abs(G_abs(:,tmp_idx)),2));
        G_abs = G_abs(:,setdiff(1:nGenAbs,tmp_idx));
        tmp_expMat = expMat_abs(:,setdiff(1:nGenAbs,tmp_idx));

        % compute reduced reachable set - compute zonotope enclosure
        tmp = prod(ones(size(tmp_expMat))-mod(tmp_expMat,2),1);
        Gquad = G_abs(:,tmp == 1);
        tmp_c = c_red + 0.5 * sum(Gquad,2);
        tmp_G = [G_abs(:,tmp == 0), 0.5*Gquad, G_red, ...
            R_err_G, R_err_GI];
        tmp_Rred = zonotope(tmp_c,tmp_G);
               
    end

% evaluate generators for current value of controller parameters and
% center shift (candPC)
nGenAbs = length(R_abs_po.G);
G_abs = cell2mat(reshape(cellfun(@(x) resolve(x,candPC,id_pc), ...
    R_abs_po.G,'UniformOutput',false),1,nGenAbs));
G_abs = G_abs(1:nx,:);

R_err = project(R_err,1:nx);

% order generators for selection
m_dep = sum(abs(G_abs),1);
[~,idx_dep] = mink(m_dep,length(G_abs));

% initial guess: user-defined order (input argument)
if length(G_abs) <= floor(nx*order)
    idx_red = [];
    return
end
red_until = length(G_abs) - floor(nx*(order-1));
idx_red = idx_dep(1:red_until);

% verify containment of the reduced set (-> recursive feasibility)
% ... compute center at candidate solution
c_red = resolve(R_abs_po.c,candPC,id_pc);
c_red = c_red(1:nx) + R_err.c; %tmp_c(1:opts.nx,:) + R_err.c;
candTermReg_Z = zonotope(candTermReg);

% adaptive case
R_red = computeReducedReachSet(c_red,G_abs,R_abs_po.E,R_err.G,R_err.GI,idx_red,nGenAbs);
if contains(candTermReg_Z,R_red,'approx:st',1e-6)
    return
end

% reduce number of to-be-reduced generators (exponential decay) until the
% containment constraint is satisfied
decay = 0.8; iter = 1;
while true
    tmp_idx = red_until*decay^iter;
    R_red = computeReducedReachSet(c_red,G_abs,R_abs_po.E,R_err.G,R_err.GI, ...
        idx_red(1:floor(tmp_idx)),nGenAbs); % to avoid infinte loop
    if contains(candTermReg_Z,R_red,'approx:st',1e-6)
        idx_red = idx_red(1:ceil(tmp_idx));
        return 
    end
    iter = iter + 1;
    if tmp_idx < 1
        return
    end
end

end


function R_abs_red = reassembleDepPart(R_abs_po,id_termReg,idx_red,nx)
% After computing the parameterizd partial zonotope over-approximation, the
% reachable set of the abstraction is stored in a struct. In this step,
% we compute the polynomial zonotope representation after reducing the
% order of the reachable set.
%
% ------------------------------------------------------------------------

% initialize re-assembly using center of center of partial zonotope
% over-approximation 
R_abs_red = R_abs_po.c;

% loop over generator
idx_nonRed = setdiff(1:length(R_abs_po.G),idx_red);
for ii = 1:length(idx_nonRed)
    tmp_idx = idx_nonRed(ii);
    % construct polyZonotope from generator
    tmp_G = [R_abs_po.G{tmp_idx}.c, R_abs_po.G{tmp_idx}.G];
    tmp_expMat = [repmat(R_abs_po.E(:,tmp_idx),1,size(tmp_G,2)); ...
        zeros(length(R_abs_po.G{tmp_idx}.id),1),R_abs_po.G{tmp_idx}.E];
     tmp_pZ = polyZonotope(zeros(nx,1), tmp_G, [], ...
       tmp_expMat,vertcat(id_termReg,R_abs_po.G{tmp_idx}.id));

    % add generator to dependent part of Rtp
    R_abs_red = exactPlus(R_abs_red,tmp_pZ);
end

end


function [R_red, mapsReduce] = computeParameterizedMaps(solver, ...
    R_abs_po,R_err,idx_red,candPC,template)
% This function computes the fuctions returning the scaling factors of the 
% reducing generators in the parameters of the reachable set (controller 
% parameters, etc.).
%
% ------------------------------------------------------------------------

% collect(to be) reduced generators
R_abs_po.G = R_abs_po.G(idx_red);
R_abs_po.E = R_abs_po.E(:,idx_red);
R_abs_po.isConstGen = R_abs_po.isConstGen(idx_red); 
R_abs_po.cVar = R_abs_po.cVar(idx_red);

% project (to be) reduced generators
R_abs_po.G = cellfun(@(x) project(x,1:solver.nx),R_abs_po.G, ...
    'UniformOutput',false);

% compute partial zonotope over-approximation of R_err
R_err = project(R_err,1:solver.nx);
% ... reachable set due to abstraction errors and disturbances
[R_err_po] = solver.paramPartZonotope( ...
    R_err,R_err.id,solver.id.var_scalar);
% remove auxiliary ids from partial zonotope over-approximation of R_err
[~,idx] = intersect(R_err_po.idv,solver.id.var_scaling,'stable');
R_err_po.idv = R_err_po.idv(idx);
R_err_po.E = R_err_po.E(idx,:);

% compute zonotope enclosure for candidate controller parameters
tmp =  cellfun(@(x) resolve(x,candPC,solver.id.var_scalar), ...
    R_abs_po.G,'UniformOutput',false);
tmp = cell2mat(reshape(tmp,1,length(R_abs_po.G)));
G = [tmp,R_err.G];

% compute initial template scaling factors 
scales = sum(abs(G),2);
% ... check if reduced generators depend on the controller parameters and 
% or the center translation (or not) - if not, and any scales(i) = 0, we 
% need no scaling factor in the ith direction since the reduced directors 
% will always have zero length in this direction 
if all(R_abs_po.isConstGen)
    [idx_sRed,~] = find(scales > 0);
    template = zonotope(template.c,template.G(:,idx_sRed));
    scales = scales(idx_sRed);
else
    % if the reduced generators depend on the controller parameters and or 
    % the center translation, we need a non-zero initial length (the
    % scaling factor can still reduce the length of this generator to zero)
    scales = max(scales, 1e-8); 
    idx_sRed = 1:length(scales);
end

% setup enclosing set 
% ... generators of enclosing set
G_red = template.G(:,idx_sRed)*diag(scales);
% ... enclosing set in augmented state space
id_red = solver.id.red(idx_sRed);
R_red = polyZonotope(R_err_po.c.c,G_red, ...
    R_err_po.GI,eye(length(id_red)),id_red);

% maps for parameterized reduction
G_map = template.G';
mPR.initScales = scales;
mPR.id = id_red;
mPR.dim = size(G_map,1);
mPR.c = polyZonotope(zeros(mPR.dim,1),[],[],...
    zeros(length(solver.id.var_scalar),0),solver.id.var_scalar);
mPR.G = [ ...
    cellfun(@(x) G_map*x,R_abs_po.G,'UniformOutput',false), ...
    cellfun(@(x) G_map*x,R_err_po.G,'UniformOutput',false)];
% find common and individual ids and setup exponent matrix accordingly
[cIdv,cIdx_dep,cIdx_ind] = intersect(R_abs_po.idv,R_err_po.idv,'stable');
[iIdv_dep,iIdx_dep] = setdiff(R_abs_po.idv,cIdv,'stable');
[iIdv_ind,iIdx_ind] = setdiff(R_err_po.idv,cIdv,'stable');
mPR.E = [R_abs_po.E(cIdx_dep,:), R_abs_po.E(cIdx_ind,:); ...
    blkdiag(R_abs_po.E(iIdx_dep,:), R_err_po.E(iIdx_ind,:))];
mPR.idv = [cIdv;iIdv_dep;iIdv_ind];
mPR.cVar = false(1,length(mPR.G));
mPR.isConstGen = [R_abs_po.isConstGen, R_err_po.isConstGen];

mapsReduce = mPR;

end
