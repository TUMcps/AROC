function alpha = computeAlpha(R0,A,B,c,xf,iter,Opts)
% COMPUTEALPHA - compute the control law for generator space control
%
% Syntax:
%       alpha = COMPUTEALPHA(R0,A,B,c,xf,iter,Opts)
%
% Description:
%       Computes the control law for one time step by solving an optimal 
%       control problem in a set based manner for the control algorithm
%       that is based on optimal control in generator space (see Eq. (12)
%       and (13) in [1]).
%
% Input Arguments:
%
%       -R0:    initial set of states (class: zonotope)
%       -A:     cell-array storing the system matrices of the linearized 
%               discrete time system
%       -B:     cell-array storing the input matrices of the linearized 
%               discrete time system
%       -c:     cell-array storing the constant offsets of the linearized 
%               discrete time system
%       -xf:    cell-array storing the goal states for all time steps
%       -iter:  current time step
%       -Opts:  a structure containing the algorithm settings
%
%           -.U:            set of admissible control inputs
%           -.N:            number of time steps
%           -.Ninter:       number of intermediate time steps
%           -.nx:           number of states
%           -.nu:           number of inputs
%
%           -.extHorizon.active:    use extended optimization horizon for 
%                                   optimal control problems
%                                   [{false} / true]
%           -.extHorizon.horizon:   length of the extended optimization
%                                   horizon in center trajectory time steps
%                                   [{'all'} / positive integer]
%           -.extHorizon.decay:     decay function for the objective
%                                   function of the optimization problem
%                                   with extended optimization horizon
%                                   [{'fall+End'} / 'uniform' / 'fall' / 
%                                    'fallLinear' / 'fallLinear+End' / 
%                                    'fallEqDiff' / 'FallEqDiff+End' / 
%                                    'rise' / 'quad' /  'riseLinear' /
%                                    'riseEqDiff' / 'end']
%
% Output Arguments:
%
%       -alpha: computed control law parameters alpha
%
% See Also:
%       generatorSpaceControl
%
% References:
%       * *[1] Schuermann et al. (2017)*, Guaranteeing constraints of 
%              disturbed nonlinear systems using set-based optimal 
%              control in generator space
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
% Authors:      Jan Wagener, Niklas Kochdumper, Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2019 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % check if an extended optimization horizon is used
    if Opts.extHorizon.active
        alpha = computeAlphaExtHorizon(R0,A,B,c,xf,iter,Opts);
    else
        alpha = computeAlpha_(R0,A{iter},B{iter},c{iter},xf{iter},iter,Opts);
    end
end


% Auxiliary Functions -----------------------------------------------------

function alpha = computeAlpha_(R0,A,B,c,xf,iter,Opts)
% compute the control law without using an extendet optimization horizon
% (see Eq. (12) and (13) in [1])

    % compute reachable set for the time-discrete system 
    % x(k+1) = A x(k) + B u(k) + c
    R = R0;
    
    for i = 1:Opts.Ninter
       R = A{i}*R + B{i}*Opts.U + c{i}; 
    end
    
    % extract center, state generators Gx, and input generator Gu
    c = center(R);
    G = generators(R);
    
    Gx = G(:,1:Opts.nx);
    Gu = G(:,Opts.nx+1:end);
    
    % initialize variables for optimization with the YALMIP toolbox
    alpha_c = sdpvar(Opts.Ninter*Opts.nu,1);
    alpha_g = sdpvar(Opts.Ninter*Opts.nu,Opts.nx,'full');
    
    % constraint |alpha_c| + sum_i |alpha_gi| < 1
    Constraints = abs(alpha_c) + sum(abs(alpha_g),2) <= 1;
    if ~Opts.refInput
        uc_iter = Opts.uc(:,(iter-1)*Opts.Ninter+1:iter*Opts.Ninter);
        ac_iter = inv(generators(Opts.U))*(uc_iter - center(Opts.U));
        Constraints = [Constraints, alpha_c == ac_iter(:)];
    end
    
    % objective function
    G_U_cell = repmat({generators(Opts.U)},Opts.Ninter,1);
    G_U = blkdiag(G_U_cell{:});
    c_U = repmat(center(Opts.U),Opts.Ninter,1);
    R_cell = repmat({Opts.R},Opts.Ninter,1);
    R_rep = blkdiag(R_cell{:});
    
    % construct "u'*R*u" part of objective function
    f_u = @(a) 2*c_U'*R_rep*G_U*a + a'*G_U'*R_rep*G_U*a;
    Objective = sum(abs(Opts.Q*(c + Gu*alpha_c - xf))) + f_u(alpha_c);
    for i = 1:Opts.nx
       Objective = Objective + sum(abs(Opts.Q*(Gx(:,i) + Gu*alpha_g(:,i)))) + ...
                   f_u(alpha_g(:,i));
    end
    
    % solve linear program with the YALMIP toolbox
    optimize(Constraints,Objective,sdpsettings('verbose',0));
    
    % store the solution
    alpha_c = value(alpha_c);
    alpha_g = value(alpha_g);
    
    alpha = mat2cell([alpha_c,alpha_g],repmat(Opts.nu,1,Opts.Ninter));
end

function alpha = computeAlphaExtHorizon(R0,A,B,c,xf,iter,Opts)
% compute the control law by using an extendet optimiaztion horizon for the
% optimal control problems

    % determine length of the extendet horizon
    len_ = Opts.N - iter + 1;
    
    if ischar(Opts.extHorizon.horizon)
        if strcmp(Opts.extHorizon.horizon,'all')
           len = len_; 
        else
           error('Wrong value for input argument "Opts.extHorizon.horizon"!');
        end
    else
        len = min(len_,Opts.extHorizon.horizon);
    end
    
    % get the weights for the decay function of the extendet horizon
    w = decayFunctions(len,Opts);

    % compute reachable set for the time-discrete system 
    % x(k+1) = A x(k) + B u(k) + c
    R = R0;
    cen = cell(len,1); Gx = cell(len,1); Gu = cell(len,1); xf_ = cell(len,1);
    counter = 1;
    
    for i = iter:iter+len-1
        
        % loop over all intermediate time steps
        for j = 1:Opts.Ninter
           R = A{i}{j}*R + B{i}{j}*Opts.U + c{i}{j}; 
        end
        
        % extract center, state generators Gx, and input generator Gu
        cen{counter} = center(R);
        G = generators(R);

        Gx{counter} = G(:,1:Opts.nx);
        Gu_ = G(:,Opts.nx+1:end);
        Gu{counter} = [Gu_,zeros(Opts.nx,Opts.Ninter*Opts.nu*len-size(Gu_,2))];
        xf_{counter} = xf{i};
        counter = counter + 1;
    end
    
    % initialize variables for optimization with the YALMIP toolbox
    alpha_c = sdpvar(Opts.Ninter*Opts.nu*len,1);
    alpha_g = sdpvar(Opts.Ninter*Opts.nu*len,Opts.nx,'full');
    
    % constraint |alpha_c| + sum_i |alpha_gi| < 1
    Constraints = abs(alpha_c) + sum(abs(alpha_g),2) <= 1;
    if ~Opts.refInput
        uc_iter = Opts.uc(:,(iter-1)*Opts.Ninter+1:(iter+len-1)*Opts.Ninter);
        ac_iter = inv(generators(Opts.U))*(uc_iter - center(Opts.U));
        Constraints = [Constraints, alpha_c == ac_iter(:)];
    end
    
    % objective function 
    Objective = 0;
    G_U_cell = repmat({generators(Opts.U)},Opts.Ninter*len,1);
    G_U = blkdiag(G_U_cell{:});
    c_U = repmat(center(Opts.U),Opts.Ninter*len,1);
    R_cell = repmat({Opts.R},Opts.Ninter*len,1);
    R_rep = blkdiag(R_cell{:});
    
    % construct "u'*R*u" part of objective function
    f_u = @(a) 2*c_U'*R_rep*G_U*a + a'*G_U'*R_rep*G_U*a;
    for i = 1:len
        
        Objective = Objective + w(i)*(sum(abs(Opts.Q*(cen{i} + Gu{i}*alpha_c - xf_{i}))) + f_u(alpha_c));

        for j = 1:Opts.nx
           Objective = Objective + w(i)*(sum(abs(Opts.Q*(Gx{i}(:,j) + Gu{i}*alpha_g(:,j)))) + f_u(alpha_g(:,j))); 
        end
    end
    
    % solve linear program with the YALMIP toolbox
    optimize(Constraints,Objective,sdpsettings('verbose',0,'solver',''));
    
    % store the solution
    alpha_c = value(alpha_c(1:Opts.nu*Opts.Ninter));
    alpha_g = value(alpha_g(1:Opts.nu*Opts.Ninter,:));
    
    alpha = mat2cell([alpha_c,alpha_g],repmat(Opts.nu,1,Opts.Ninter));
end