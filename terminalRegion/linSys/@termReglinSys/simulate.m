function trajectory = simulate(T,y0,w,v)
% SIMULATE - simulate a trajectory of a terminal region controller
%
% Syntax:
%       trajectory = SIMULATE(obj,y0,w,v)
%
% Description:
%       Simulate a trajectory of a linear system controlled with the 
%       safety-preserving controller associated with a terminal region 
%       computed with the linSysContainment algorithm for random initial 
%       points inside the terminal region and randomly drawn disturbance 
%       values.
%
% Input Arguments:
%
%       -y0:     initial point for simulation (class: double)
%
%       -w:      matrix storing the values for the disturbances over time 
%                (dimension: [T.optsInternal.nw,T.Opts.N])
%       -v:      matrix storing the values for the measurement errors
%                over time (dimension: [T.optsInternal.nv,T.Opts.N+1])
%
% Output Arguments:
%       -trajectory: struct containing all parts of the simulated
%                    trajectory, i.e., x, y, yexact, u, v, w, t 
%
% See Also:
%       termRegNonlinSysLinApproach, simulateRandom, computeTermRegNonlinSysLinApproach
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Syntesis developed at the Chair of Robotics, 
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
% Copyright (c) 2024 Chair of Robotics, Arificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------   

% Check input parameters
inputArgsCheck({{y0,'att','numeric',{'vector','nonnan','finite'}}});

inputArgsCheck({{w,'att','numeric',{'nonnan','finite'}}});
inputArgsCheck({{v,'att','numeric',{'nonnan','finite'}}});

if size(w,2) ~= T.Opts.N
    error('The array of disturbances w does not have the right length (should be equal to Opts.N).')
end
if size(v,2) ~= (T.Opts.N+1)
    error('The array of disturbances v does not have the right length (should be equal to Opts.N+1).')
end

dims = T.optsInternal.dynDims;
sys = T.optsInternal.sys;

N = T.Opts.N;
dt = T.Opts.timeStep;

equalDimCheck(T.Y, y0);
equalDimCheck(T.W, w);
equalDimCheck(T.V, v);

y0_iter = y0;
x0_iter = y0-T.F*v(:,1)-T.gamma;

traj_t = [0];
traj_x = [x0_iter];
traj_y = [y0_iter];
traj_yexact = [x0_iter+T.gamma];
traj_u = [zeros([dim(T.U) 1])];
traj_v = [v(:,1)];
traj_w = [zeros([dim(T.W) 1])];

if strcmp(T.Opts.terminalRegionSet, 'ellipsoid')
    if T.feedback_speedup.is_bijective
        % If the terminal region is just an ellipsoid, things are
        % really very easy
        preimage = T.feedback_speedup.multFactor * y0 + T.feedback_speedup.constFactor;

        beta = preimage(1:T.optsInternal.size_s);
        beta_V = preimage((T.optsInternal.size_s+1):end);
    else
        % Otherwise, we have to compute a slightly costly quadratic program
        V_ngen = size(generators(T.optsInternal.Param.V),2);

        H = 2.*blkdiag(speye(T.optsInternal.size_s), sparse(V_ngen,V_ngen));
        f = sparse(T.optsInternal.size_s+V_ngen,1);

        A = [sparse(V_ngen,T.optsInternal.size_s) speye(V_ngen); sparse(V_ngen, T.optsInternal.size_s) -speye(V_ngen)];
        b = ones([2*V_ngen 1]);

        Aeq = T.feedback_speedup.comb_G;
        beq = sparse(y0 - T.feedback_speedup.comb_c);


        persistent options_quadprog
        if isempty(options_quadprog)
            options_quadprog = optimoptions('quadprog', 'Display', 'none');
        end
        [preimage, fval, exitflag] = quadprog(H,f,A,b,Aeq,beq,[],[],options_quadprog);

        tol = 1e-5;

        % If the problem is not feasible, y has been chosen incorrectly
        if exitflag == -2 || fval > 1 + tol
            error('The point x does not lie in the corresponding reachable set.')
            % In case anything else went wrong, throw out an error
        elseif exitflag ~= 1
            throw(CORAerror('CORA:solverIssue'));
        end

        beta = preimage(1:T.optsInternal.size_s);
        beta_V = preimage((T.optsInternal.size_s+1):end);
    end

else
    % Suppress solver output
    persistent options_linprog
    if isempty(options_linprog)
        options_linprog = optimoptions('linprog', 'Display', 'none');
    end

    % See also cora/contSet/@zonotope/zonotopeNorm
    % Retrieve dimensions of the generator matrix
    n = size(T.feedback_speedup.comb_G, 1);
    m = size(T.feedback_speedup.comb_G, 2);

    % Set up objective and constraints of the linear program
    f = [1;sparse(m,1)];

    Aeq = [sparse(n, 1) T.feedback_speedup.comb_G];
    beq = y0 - T.feedback_speedup.comb_c;

    Aineq1 = [-ones([m 1]) speye(m)];
    Aineq2 = [-ones([m 1]) -speye(m)];

    Aineq = [Aineq1; Aineq2];
    bineq = sparse(2*m, 1);

    % Solve the linear program
    [preimage, fval, exitflag] = linprog(f, Aineq, bineq, Aeq, beq, [], [], options_linprog);

    tol = 1e-5;

    % If the problem is not feasible, y has been chosen incorrectly
    if exitflag == -2 || fval > 1 + tol
        error('The point x does not lie in the corresponding reachable set.')
        % In case anything else went wrong, throw out an error
    elseif exitflag ~= 1
        throw(CORAerror('CORA:solverIssue'));
    end

    beta = preimage(2:(m - size(generators(T.optsInternal.Param.V),2)+1));
    beta_V = preimage((m - size(generators(T.optsInternal.Param.V),2)+2):end);
end


for i = 1:N
    % Compute input that has to be applied
    u_iter = computeControlInput(T, y0_iter, i, beta, beta_V);

    f = @(s,z) T.dynamics(z,u_iter,w(:,i));
    [t_iter,  x_iter] = ode45(f,[(i-1)*dt i*dt],x0_iter);

    x_iter = x_iter';
    y_iter = x_iter + sys.F * v(:,i+1) + sys.gamma;
    yexact_iter = x_iter + sys.gamma;

    traj_t = [traj_t t_iter'];
    traj_x = [traj_x x_iter];
    traj_y = [traj_y y_iter];
    traj_yexact = [traj_yexact yexact_iter];
    traj_u = [traj_u kron(u_iter, ones([1 length(t_iter)]))];

    % For the case where we need to verify the trajectory
    traj_v = [traj_v kron(ones([1 length(t_iter)]), v(:,i+1))];
    traj_w = [traj_w kron(ones([1 length(t_iter)]), w(:,i))];

    y0_iter = y_iter(:,end);
    x0_iter = x_iter(:,end);
    u_iter = u_iter;
end

% Saving
trajectory.x = traj_x;
trajectory.y = traj_y;
trajectory.yexact = traj_yexact;
trajectory.u = traj_u;
trajectory.v = traj_v;
trajectory.w = traj_w;
trajectory.t = traj_t;

