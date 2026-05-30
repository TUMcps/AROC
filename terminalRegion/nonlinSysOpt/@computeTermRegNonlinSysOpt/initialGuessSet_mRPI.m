function minRPIset = initialGuessSet_mRPI(obj,dynamics,Q,R,varargin)
%INITIALGUESSSET_MRPI - computes an (invariant) outer approximation of the 
%       minimal robust positively invariant set of the linearized dynamics.
%
% Syntax:
%       minRPIset = initialGuessSet_mRPI(obj,dynamics,Q,R)
%       minRPIset = initialGuessSet_mRPI(obj,dynamics,Q,R,flag_invariant)
%
% Description:
%       To obtain an initial guess for the optimization-based approaches 
%       for computing terminal regions of nonlinear systems, we compute an 
%       outer approximation of the minimal robust positvely invariant 
%       (mRPI) set of the linearized dynamics (based on [1, Alg. 2]). 
%       By default, the loop in [1, Alg. 2, line 10-16] is omitted and the
%       output order of initGuess is set to a user-defined value (see 
%       description of input arguments). If the additional input argument 
%       flag_invariant is set to true, this loop is executed.
%
% Input Arguments:
%       -obj:               instance of class
%                           computeTermRegNonlinSysOpt
%       -dynamics:          instance of class termRegNonlinSysDynamics
%       -Q:                 state weighting matrix
%                           (positive-definite matrix of dimension 
%                           [obj.nx,obj.nx])
%       -R:                 control input weighting matrix
%                           (positive-definite matrix of dimension 
%                           [obj.nu,obj.nu])
%       -flag_invariant:    (optional), see description - if omitted or 
%                           false, the user-defined order is chosen as
%                           opts.termRegOrder
%                           (logical)
%
% Output Arguments:
%       -minRPIset:     outer approximation of the mRPI set
%                       (class: termRegNonlinSysLinApproach)
%           
% See also:
%       -
%
% References:
%       [1] Gruber et al., "Computing safe sets of linear sampled-data 
%           systems", 2021, IEEE Control Systems Letter, vol. 5, no. 2, 
%           pp. 385-390.
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

% extract optional input arguments
if isscalar(varargin) || length(varargin) == 3
    flag_invariant = varargin{end};
    assert(islogical(flag_invariant) || ismember(flag_invariant,[0,1]), ...
        'Optional input argument flag_invariant must be logical.')
else
    flag_invariant = false;
end

% get linearized system
AB_d_nom = dynamics.AB_d.C;
Ad = AB_d_nom(:,1:obj.nx);
Bd = AB_d_nom(:,obj.nx+(1:obj.nu));
RlinW = dynamics.RlinW_dt_nom;
assert(contains(RlinW,zeros(obj.nx,1)), ...
    'Reachable set due to the set of disturbances must contain the origin!')

% compute LQR gain 
[Kminus,~,~] = dlqr(Ad,Bd,Q,R); K = -Kminus;

% closed-loop system
Afb = Ad+Bd*K;

% approximate mRPI of linearized system ----------------------------------
% outer-approximation of mRPI, see Alg. 2 'Safe terminal set' in [1]
if ~isBounded(obj.H_X)
    warning(['Set of admissible states is unbouded. ' ...
        'Set unit box as initial set for outer reachable set.'])
    width = ones(obj.nx,1);
else
    width = min([interval(obj.H_X).sup-obj.xEq, obj.xEq-interval(obj.H_X).inf],[],2);
end
Rout{1} = zonotope(interval(obj.xEq-width, obj.xEq+width)) + zonotope(-obj.xEq);
Rin{1} = zonotope(zeros(obj.nx,1));

reductionTechnique = 'pca';

while true
    Rout{end+1} = ...
        reduce(Afb*Rout{end}+RlinW,reductionTechnique,100);
    Rin{end+1} = ...
        reduce(Afb*Rin{end}+RlinW,reductionTechnique,100);
      
    % (approximate) Haussdorff distance between RnextIn and RnextOut
    RnextIn_box = polytope(box(Rin{end}));
    d = -min(RnextIn_box.b - RnextIn_box.A*center(Rout{end}) ...
        - abs(RnextIn_box.A*generators(Rout{end}))*...
        ones(size(generators(Rout{end}), 2), 1));

    if d < 1e-3
        break
    end
end

% order reduction - if flag_invariant = true, the order is increased until
% an invariant outer-approximation is found or the order of Rout{end} is
% exceeded
if flag_invariant
    maxOrder = (size(Rout{end}.G,2)/obj.nx);
    for order = obj.termRegOrder:maxOrder
        initGuess = reduce(Rout{end},reductionTechnique,order);
        R = Afb*initGuess + RlinW;
        if contains(initGuess,R,'approx:st',1e-6)
            break
        elseif ~contains(initGuess,R,'approx:st',1e-6) && order == maxOrder
            error('Found no invariant outer approximation of the minimal RPI set.')
        end
    end
    
else
    order = obj.termRegOrder;
end

% translate initial guess to the equilibrium state
termReg = reduce(Rout{end},reductionTechnique,order) + obj.xEq;

% assign output argument
termRegCtrl = zonotope(obj.uEq, K*termReg.G);
linDynamicsHandle = @(x,u,w) obj.xEq + Ad*(x-obj.xEq) + Bd*(u-obj.uEq) + w;
minRPIset = termRegNonlinSysLinApproach(linDynamicsHandle,termRegSysType.DT, ...
    termReg,termRegCtrl,dynamics.dt,obj.H_U,RlinW);

end
