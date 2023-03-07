function [Rfin,reachSet]=reachLinearController(dynamic,R0,cu,Gu,inputAssign,Opts)
% REACHLINEARCONTROLLER - reachabilty analysis for the linear approximation
%                         of the exact control law
%
% Syntax:
%       [Rfin,reachSet] = REACHLINEARCONTROLLER(dynamic,R0,cu,Gu,inputAssign,Opts)
%
% Description:
%       Computes the reachable set at the end of one timestep of the
%       center trajectory as well as all reachable sets during this
%       timestep.
%
% Input Arguments:
%
%       -dynamic:       object containing the system dynamics (class:
%                       nonlinearSys) 
%       -R0:            extended initial zonozope, which contains the state  
%                       as well as the auxiliary states (dimension: 2*nx)
%       -cu:            cell array containing the center vectors of the
%                       input zonotopes for each intermediate timestep
%                       (dimension cell array: Opts.Ninter, 
%                        diemnsion entries: [nu,1])
%       -Gu:            cell array containing the generator matrices of the
%                       input zonotopes for each intermediate timestep
%                       (dimension cell array: Opts.Ninter, 
%                        diemnsion entries: [nu,nx])
%       -inputAssign:   Matrix that encode the information which generator
%                       of the first input zonotope belongs to which
%                       generator of the initial state zonotope R0
%                       (dimension: [nx,zonotope order of R0]) 
%       -Opts:          structure containing user defined options for the 
%                       algorithm   
%
% Output Arguments:
%
%       -Rfinal:    final zonotope at the end of reachability analysis
%       -reachSet:  cell array containing the reachable sets of all
%                   intermediate time steps
%
% See Also:
%       convexInterpolationControl, computeLinearController
%
% References:
%       * *[1] Schuermann et al. (2017)*, Convex interpolation control with 
%              formal guarantees for disturbed and constrained nonlinear 
%              systems
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
% Authors:      Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

reachSet = [];
tStart = Opts.tStart;

% get reachability parameter and settings
options = Opts.ReachOpts;
params = Opts.ReachParams;

% extract center and generators for the initial set
c_R0 = center(R0);

if isa(R0,'zonotope') 
    G_R0 = generators(R0);
else
    G_R0 = [R0.G,R0.Grest];
    E_R0 = R0.expMat;
end

% reachable set is computed for one time step of the center trajectory, 
% which consists of N_intersteps intermediate steps
for l = 1:Opts.Ninter    
 
    % generate the extended zonotope which consists of the state
    % generators, the input generators and the inputAssign vectors, which
    % contain the information which inputs belong to this vector, when we
    % have to change the input after each intermediate step. This is all
    % used to make reachability analysis simpler
    
    % construct extended zonotope
    if isempty(Opts.V)
        c_big = [c_R0;cu{l};zeros(size(inputAssign,1),1)];
        G_big = [G_R0;Gu{l}*inputAssign;inputAssign];
    else
        c_big = [c_R0;zeros(Opts.nx,1)];
        G_big = [G_R0; inputAssign];
    end
    
    if isa(R0,'zonotope')
        R0 = zonotope(c_big,G_big);
    else
        R0 = polyZonotope(c_big,G_big(:,1:size(E_R0,2)), ...
                          G_big(:,size(E_R0,2)+1:end),E_R0); 
    end 
    
    % add set of measurement errors
    if ~isempty(Opts.V)
       R0 = cartProd(R0,Opts.V);
       p = [cu{l}; reshape(Gu{l},[Opts.nu*Opts.nx,1])];
       params.paramInt = p;
    end
    
    % update initial set and time for reachability analysis
    params.R0 = R0;
    if tStart ~= 0
        params.tStart = tStart;
        params.tFinal = params.tStart + Opts.hinter;
    else
        params.tFinal = Opts.hinter;
    end

    % compute reachable set
    Rtemp = reach(dynamic,params,options);

    % store reachble set and update time
    reachSet = add(reachSet,Rtemp);
    tStart = params.tFinal;
    
    % update initial set
    Rfin = Rtemp.timePoint.set{end};
    
    if isa(Rfin,'zonotope')
        G_ = generators(Rfin);
    else
        G_ = [Rfin.G,Rfin.Grest];
        E_R0 = Rfin.expMat;
    end

    c_ = center(Rfin);
    c_R0 = c_(1:Opts.nx);
    G_R0 = G_(1:Opts.nx,:);

    if isempty(Opts.V)
        inputAssign = G_(Opts.nx+Opts.nu+1:Opts.nx+Opts.nu+Opts.nx,:); 
    else
        inputAssign = G_(Opts.nx+1:2*Opts.nx,:); 
    end
end