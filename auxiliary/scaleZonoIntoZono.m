function scale = scaleZonoIntoZono(candInbody,circumbody)
% SCALEZONOINTOZONO - computes the biggest scaling factor so that the 
%       uniformly scaled candInbody is contained in circumbody.
%
% Syntax:
%       scale = scaleZonoIntoZono(candInbody,circumbody)
%
% Description:
%       This function computes the biggest scaling factor alpha so that the 
%       uniformly scaled candInbody (the center is fixed) is still 
%       contained in circumbody, i.e.,
%           zonotope(candInbody.c,candInbody.G*alpha) \subseteq
%           zonotope(circumbody.c,circumbody.G).
%       The containment problem is encoded using the sufficient conditions 
%       in [1, Corollary 4].
%
% Input Arguments:
%       - candInbody:   set to be contained 
%                       (class: zonotope)
%
%       - circumbody:   set to contain the scaled set candInbody 
%                       (class: zonotope)
%
% Output Arguments:
%       - scale:    scaling factors for the generators of candInbody
%                   (positive scalar)
%
% See Also:
%       terminalRegion, termRegNonlinSysLinApproach
%
% References:
%       [1] S. Sadraddini and R. Tedrake, "Linear Encodings for Polytope 
%           Containment Problems," IEEE Conference on Decision and Control, 
%           2019, pp. 4367-4372.
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
% Copyright (c) 2021 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 

% check input arguments
if ~isa(candInbody,'zonotope')
    error('Candidate inbody must be represented as a zonotope!');
end
if ~isa(circumbody,'zonotope')
    error('Circumbody must be represented as a zonotope!');
end

% declare optimization problems
% ... scaling factor
var_s = sdpvar(1,1);
% ... auxiliary variables for encoding the containment constraint
var_Gamma = sdpvar(size(circumbody.G,2),size(candInbody.G,2),'full');
var_gamma = sdpvar(size(circumbody.G,2),1,'full');

% cost function - maximize the scaling factor
cost = -var_s;

% constraints - see [1, Corollary 4]
constraints =  [candInbody.G*var_s == circumbody.G*var_Gamma, ...
    circumbody.c - candInbody.c == circumbody.G*var_gamma, ...
    sum(abs([var_gamma,var_Gamma]),2) <= 1];
% constraints - var_s must be positive
constraints = [constraints, var_s >= 0];

% solve linear program
options = sdpsettings('solver','linprog','verbose',0','allownonconvex','false');
diagnostics = optimize(constraints,cost,options);


if diagnostics.problem == 0
    scale = value(var_s);
else
    error(['Failed to scale candidate indbody so that the scaled set ' ...
        'is a subset of the circumbody!'])
end

end
