function [R0opt,RcontOpt] = scaleInitialSet(sys,R0,H,goalSet,U,iter,Opts)
% SCALEINITIALSET - scale the initial set so the goal set is reached
%
% Syntax:
%       [R0opt,RcontOpt] = SCALEINITIALSET(sys,R0,H,goalSet,U,iter,Opts)
%
% Description:
%       This function scales the initial set for reachability analysis in
%       such a way that the final reachable set is located inside the goal
%       set.
%
% Input Arguments:
%
%       -sys:       object that represents the dynamics of the closed-loop 
%                   system (class: nonlinParamSys)
%       -R0:        initial set (class: zonotope)
%       -H:         matrix storing the generator-to-input assignment
%       -goalSet:   goal set that should be reached (class: zonotope)
%       -U:         cell-array containing the sets of admissble control
%                   inputs for all intermediate time steps
%       -iter:      index of the current time step
%       -Opts:      structure containing the following options
%
%           -.iter:     number of iterations for optimization
%
% Output Arguments:
%
%       -R0opt:     scaled initial set (class: zonotope)
%       -RcontOpt:  cell-array storing the time interval reachable set for
%                   the final solution
%
% See Also:
%       safetyNetControl
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
% Authors:      Moritz Klischat, Niklas Kochdumper
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization
    s = []; RcontOpt = []; R0opt = []; R0_ = R0;

    % convert goal set to halfspace representation
    goalSet = mptPolytope(goalSet);
    
    if ~isempty(Opts.V) && iter < Opts.N
       goalSet = minus(goalSet,Opts.V,'approx'); 
    end
    
    C = get(goalSet,'A');
    d = get(goalSet,'b');
    
    % compute reachable set for the unscaled set
    [Rfin,~] = reachSafetyNetContr(sys,R0,H,U,Opts);
    
    % loop over all iterations
    for i = 1:Opts.iter   
        
        % scale the initial set by solving a optimization problem
        s_ = factorScaling(R0_,Rfin,H,C,d,Opts.stateCon);
            
        if isempty(s)
           s = s_; 
        else
           s = s.*s_;
        end
       
        % compute scaled initial set and scaled correspondence matrix H
        R0_ = zonotope([center(R0),generators(R0)*diag(s)]);
        
        % compute forward reachable set with scaled initial set
        [Rfin,Rcont] = reachSafetyNetContr(sys,R0_,H,U,Opts);
        
        % check if the constraints are satisfied
        if contains(goalSet,Rfin)
            if isempty(R0opt) || volume(R0opt) < volume(R0_)
                R0opt = R0_;
                RcontOpt = Rcont;
            end
        end
    end
    
    % check if a suitable solution could be found
    if isempty(R0opt)
       error('Failed to find a suitable solution!'); 
    end
end