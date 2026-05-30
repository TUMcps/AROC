function T = computeTermReglinSys(benchmark, Param, Opts)
% COMPUTETERMREGLINSYS - compute terminal region with
%       linear system approach, using only containment
%
% Syntax:
%       T = COMPUTETERMREGLINSYS(benchmark, Param, Opts)
%
% Description:
%       This function computes a terminal region with the approach in [1]
%       and [2].
%
% Input Arguments:
%       -benchmark:  name of the considered benchmark model (see
%                    "aroc/benchmarks/dynamics/...")
%
%       -Param:      a structure containing the benchmark parameters
%           -.U:     set of admissible control inputs
%           -.W:     set of uncertain disturbances
%           -.V:     set of measurement noise
%           -.X:     set of state constraints
%           -.Y:     set of output constraints
%           -.Yexact:set of error-free output constraints
%           -.F, -.gamma:
%                    defines the output equation
%                    y = x + Fv + gamma
%           U, X, and Y need to be polytopes, or will be transformed to 
%           polytopes within the algorithm (i.e., intervals or zonotopes
%           can also be used here, but will be transformed to polytopes).
%           W, V, and Ustart on the other hand need to be intervals or
%           zonotopes.
%
%       - Opts: a structure containing the algorithm settings
%           -.N:                  number of time steps 
%                                 [{10} / positive integer]
%           -.timeStep:           sampling time [{0.1} / positive scalar]
%           -.terminalRegionType: method used for computing the terminal
%                                 region [{'implicit'} or 'fixed']
%           -.terminalRegionSet:  set type of the terminal region
%                                 [{'zonotope'} or 'ellipsoid']
%           -.taylorOrder:        Taylor order for certain approximations
%                                 needed during the reachability analysis
%                                 [{5} / positive integer]
%           -.genMethod:          method for computing the fixed directions
%                                 [{'spherical'} or 'provided']
%           -.nGenerators:        if .genMethod='spherical', .nGenerators
%                                 should be the number of generators for
%                                 the fixed generator matrix
%                                 [{dim(X)}/ positive integer]
%           -.G:                  if .genMethod='provided', .matrix should
%                                 be the desired matrix for the fixed
%                                 generators
%           -.K:                  feedback matrix for the terminal
%                                 controller
%           -.Q:                  state weighting matrix for the LQR
%                                 approach applied to determine the
%                                 terminal controller
%                                 [{eye(nx)} / positive definite
%                                 square matrix]
%           -.R:                  input weighting matrix for the LQR
%                                 approach applied to determine the
%                                 terminal controller
%                                 [{eye(nu)} / positive definite
%                                 square matrix]
%
% Output Arguments:
%       -T:     object of class termRegContainmentLinSys
%
% See Also:
%       terminalRegion, termRegLinSysApproach
%
% References:
%       * *[1] "Approximability of the Containment Problem for Zonotopes
%              and Ellipsotopes", A. Kulmburg et al., submited to TAC
%              in 2024
%       * *[2] Gruber et al. (2021)*, Computing safe sets of linear 
%              sampled-data systems, IEEE Control Syst. Lett., vol. 5, 
%              no. 2, pp. 385-390
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
% Authors:      Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

%% Emptying Yalmip variables, since this seems to change quite a lot
yalmip('clear')

%% Pre-Processing
tic;
optsInternal = setOptsInternal(benchmark,Opts,Param);
Opts = optsInternal.Opts;
Param = optsInternal.Param;

% If we are using the 'fixed' algorithm, we need to compute the terminal
% region first
if strcmp(Opts.terminalRegionType, 'fixed')
    termSet = fixed_terminalSet(optsInternal);
else
    termSet = [];
end
t_preprocessing = toc;


%% Setting up reachability
tic;

Rtp = cell([1 Opts.N+1]);
Rti = cell([1 Opts.N]);
ctrl_variable = cell([1 Opts.N]);
utotal = cell([1 Opts.N]);
additional_variables = cell([1 Opts.N]);

s = sdpvar(optsInternal.size_s, 1, 'full');
cost = -geomean(s');

constraints = [s >= sparse(optsInternal.size_s, 1)];

X0 = optsInternal.preH * diag(s) * optsInternal.postH;
c = sdpvar(optsInternal.stdDynDims.nx, 1, 'full');

% Preparing everything for the initial step
Rtp{1}.X.G = X0;
Rtp{1}.X.c = c;
% We now set up Yexact and Y
Rtp{1}.Yexact.G = X0;
Rtp{1}.Yexact.c = c + optsInternal.stdSys.gamma;
Rtp{1}.Y.G = [Rtp{1}.Yexact.G optsInternal.stdSys.F];
Rtp{1}.Y.c = Rtp{1}.Yexact.c;

for k=1:Opts.N
    [Rtp_k, Rti_k, ctrl_variable_k, utotal_k, additional_variables_k, additional_constraints_k] = implicit_oneStepReachability(Rtp{k}, optsInternal);
    Rtp{k+1} = Rtp_k;
    Rti{k} = Rti_k;
    ctrl_variable{k} = ctrl_variable_k;
    utotal{k} = utotal_k;
    additional_variables{k} = additional_variables_k;

    constraints = [constraints additional_constraints_k];
end
    
t_reachability = toc;

%% Setting up state, input, and containment constraints

tic;
if strcmp('implicit', Opts.terminalRegionType)
    [variables_sic, constraints_sic] = implicit_stateInputContainment(optsInternal, Opts.terminalRegionSet, Rtp, Rti, utotal, optsInternal.preH, s);
elseif strcmp('fixed', Opts.terminalRegionType)
    [variables_sic, constraints_sic] = fixed_stateInputContainment(optsInternal, Rtp, Rti, utotal, termSet);
end
constraints = [constraints constraints_sic];
t_state_input_containment_constraints = toc;

%% Solving the optimization problems

tic;

yalmipDiagnostics = optimize(constraints,cost,optsInternal.yalmipOptions);

if yalmipDiagnostics.problem ~= 0
    warning("The solver had problems computing the result of the optimization problem. It is thus very likely that the result will be false. Here is some more detailed information:"+sprintf('\n')+ yalmipDiagnostics.info)
end

% Evaluate all results
s = value(s);
c = value(c);
for i=1:(Opts.N+1)
    Rtp{i}.X.G = value(Rtp{i}.X.G);
    Rtp{i}.X.c = value(Rtp{i}.X.c);
    Rtp{i}.Yexact.G = value(Rtp{i}.Yexact.G);
    Rtp{i}.Yexact.c = value(Rtp{i}.Yexact.c);
    Rtp{i}.Y.G = value(Rtp{i}.Y.G);
    Rtp{i}.Y.c = value(Rtp{i}.Y.c);
end

for i=1:Opts.N
    Rti{i}.X.G = value(Rti{i}.X.G);
    Rti{i}.X.c = value(Rti{i}.X.c);
    Rti{i}.Yexact.G = value(Rti{i}.Yexact.G);
    Rti{i}.Yexact.c = value(Rti{i}.Yexact.c);
    Rti{i}.Y.G = value(Rti{i}.Y.G);
    Rti{i}.Y.c = value(Rti{i}.Y.c);

    ctrl_variable{i}.U_X = value(ctrl_variable{i}.U_X);
    ctrl_variable{i}.U_V = value(ctrl_variable{i}.U_V);
    ctrl_variable{i}.u_c = value(ctrl_variable{i}.u_c);
end

R = zonotope(Rtp{end}.Y.c, Rtp{end}.Y.G);

if strcmp(Opts.terminalRegionSet, 'ellipsoid')
    RCI = (optsInternal.preH * diag(s)) * ellipsoid(speye(optsInternal.size_s)) + c + optsInternal.stdSys.gamma;
    RCI_zono = zonotope(c + optsInternal.stdSys.gamma, optsInternal.preH * diag(s) * optsInternal.postH);

    RCIState = (optsInternal.preH * diag(s)) * ellipsoid(speye(optsInternal.size_s)) + c;
    RCIState_zono = zonotope(c, optsInternal.preH * diag(s) * optsInternal.postH);
else
    RCI = zonotope(c + optsInternal.stdSys.gamma, optsInternal.preH * diag(s) * optsInternal.postH);
    RCI_zono = RCI;

    RCIState = zonotope(c, optsInternal.preH * diag(s) * optsInternal.postH);
    RCIState_zono = RCIState;
end

if strcmp(Opts.terminalRegionType, 'fixed')
    termSet = zonotope(termSet.c, termSet.G);
else
    termSet = [];
end

t_solver = toc;

%% Construct the terminal region object

runtimes.preprocessing = t_preprocessing;
runtimes.reachability = t_reachability;
runtimes.state_input_containment_constraints = t_state_input_containment_constraints;
runtimes.solver = t_solver;
runtimes.total = t_preprocessing + t_reachability + t_state_input_containment_constraints + t_solver;

T = termReglinSys(optsInternal.dynamics, RCI, RCI_zono, RCIState, RCIState_zono, termSet, Rtp, R, s, ctrl_variable, runtimes, optsInternal);

end
