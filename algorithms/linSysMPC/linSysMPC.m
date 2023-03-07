function res = linSysMPC(benchmark,Param,Opts)
% LINSYSMPC - Model Predictive Control for linear systems
%
% Syntax:
%       res = LINSYSMPC(benchmark,Param,Opts)
%
% Description:
%       Implementation of the Model Predictive Control algorithm for linear
%       sampled-data systems described in [1]
%
% Input Arguments:
%
%       -benchmark:    name of the considered benchmark model (see
%                      "aroc/benchmarks/dynamics/...")
%       -Param:             a structure containing the benchmark parameters
%
%           -.x0:           initial state
%           -.xf:           goal state
%           -.U:            set of admissible control inputs (class:
%                           interval)
%           -.W:            set of uncertain disturbances (class: interval 
%                           or zonotope)
%           -.V:            set of measurement errors (class: interval or
%                           zonotope)
%           -.X:            set of state constraints (class: mptPolytope)
%
%       -Opts:              a structure containing the algorithm settings
%
%           -.tOpt:         final time for the optimization  
%           -.N:            number of time steps
%                           [{10} / positive integer]
%           -.termReg:      terminal region around the steady state xf
%                           (class: mptPolytope or terminalRegion)
%           -.Q:            state weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.R:            input weighting matrix for the cost function of
%                           optimal control problem (reference trajectory)
%           -.K:            feedback matrix for the feedback controller
%           -.Qlqr:         state weighting matrix for the cost function of
%                           the LQR approach (feedback controller)
%           -.Rlqr:         input weighting matrix for the cost function of
%                           the LQR approach (feedback controller)
%           -.realTime:     flag specifying if real time computation time 
%                           constraints are considered (Opts.realTime = 1) 
%                           or not (Opts.realTime = 0) [{true} / boolean]
%           -.alpha:        contraction rate for the contraction constraint
%                           [{0.1} / alpha > 0]
%
%           -.cora.taylorTerms:         taylor order for computing e^At
%                                       [{10} / positive integer]
%           -.cora.zonotopeOrder:       upper bound for the zonotope order
%                                       [{50} / positive integer]
%
% Output Arguments:
%
%       -res:       results object storing the computed reachable set and
%                   the center trajectory
%
% See Also:
%       reachsetMPC
%
% References:
%       * *[1] F. Gruber and M. Althoff, “Scalable robust model predictive
%              control for linear sampled-data systems," CDC 2019
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
% Authors:      Max Beier, Laura Lützow, Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % Initialization ------------------------------------------------------

    % parse parameter and options
    Opts = loadParamAndOpts(benchmark,Param,Opts);
    
    % initialize optimization problem
    [optimizer,optimizerInit] = initOptProblem(Opts);

    % intitialize variables
    x = Opts.x0; t = 0;
    sim.x = []; sim.t = []; sim.u = [];
    cnt = 1;
    Rall = {zonotope(x)}; RallCont = {};


    % Initial Solution ----------------------------------------------------

    % solve initial optimization problem
    [res,flag] = optimizerInit({x});

    if flag ~= 0
        error('Failed to find a feasible initial solution!'); 
    end

    u = res{1}; p = res{2}; J = sum(p);

    % compute reachable set
    [R,Rcont] = compReachSet(x,u,Opts);


    % Algorithm -----------------------------------------------------------

    while ~contains(Opts.termReg, x)

        % try to find a better solution via optimization
        clock = tic;
        [res,flag] = optimizer({x,u(:,cnt),J});
        tComp = toc(clock);

        u_ = res{1}; p_ = res{2};

        % simulate the system for random disturbances
        w = randPoint(Opts.param.U,10); 
        v = randPoint(Opts.V); v = v(1:Opts.nx);

        simOpts.x0 = [x; Opts.K*(x - Opts.xf + v) + u(:,cnt)];
        simOpts.tFinal = Opts.dt;
        simOpts.u = w;

        [tTraj,xTraj] = simulate(Opts.linearSystem,simOpts);

        sim.t = [sim.t; tTraj + t]; sim.x = [sim.x; xTraj(:,1:Opts.nx)]; 
        sim.u = [sim.u; xTraj(:,Opts.nx+1:end)];

        t = t + Opts.dt; x = xTraj(end,1:Opts.nx)';

        % store reachable set
        Rall{end+1} = R{cnt+1}; RallCont = add(RallCont,Rcont(cnt));

        % switch to new solution if it is feasible and better than old one
        if flag == 0 && (~Opts.realTime || tComp < Opts.dt)
            disp('Switch to new solution');
            u = u_(:,2:end); p = p_; cnt = 1;
            [R,Rcont] = compReachSet(xTraj(1,1:Opts.nx)',u_,Opts);
        else
            disp('Keep old solution');
            cnt = cnt + 1;
        end

        J = sum(p(cnt:end));
    end


    % Output Arguments ----------------------------------------------------

    % store final reachable set
    Rall{end+1} = R{cnt+1}; RallCont = add(RallCont,Rcont(cnt));

    % construct results object
    res = results(RallCont,Rall,[],{sim});
end


% Auxiliary Functions -----------------------------------------------------

function [R,Rcont] = compReachSet(x0,u,Opts)
% compute reachable set for the entire optimization horizon

    % initialization
    R = cell(Opts.N,1);
    Rcont = [];
    param = Opts.param;
    R0 = zonotope([x0; zeros(Opts.nu,1)]) + Opts.V;

    % loop over all time steps
    for i = 1:Opts.N
        param.R0 = Opts.XKX*(R0 + Opts.V);
        param.R0 = param.R0 + [zeros(Opts.nx,1); u(:,i) - Opts.K*Opts.xf];
        Rtmp = reach(Opts.linearSystem,param,Opts.cora);
        R0 = Rtmp.timePoint.set{end};
        R{i} = project(Rtmp.timePoint.set{1},1:Opts.nx);
        Rcont = add(Rcont, project(Rtmp,1:Opts.nx));
    end
end