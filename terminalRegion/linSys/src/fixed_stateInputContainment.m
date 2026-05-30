function [variables, constraints] = fixed_stateInputContainment(optsInternal, Rtp, Rti, utotal, terminalSet)
% FIXED_STATEINPUTCONTAINMENT - sets up the containment-based constraints
%       on the state and input for the linSys algorithm, with fixed
%       terminal set
%
% Syntax:
%       [variables, constraints] = fixed_stateInputContainment(optsInternal, containmentType, Rtp, Rti, utotal, preH, s)
%
% Input Arguments:
%       Rtp:               cell array of structs describing the time-point
%                          solutions
%
%       Rti:               cell array of structs describing the
%                          time-interval solutions
%
%       utotal:            cell array of structs describing the input
%
%       terminalSet:       terminal set, as computed by fixed_terminalSet
%
% Description:
%       This function prepares the necessary constraints needed to solve
%       the optimization problem for the containmentLinSys algorithm,
%       assuming one uses Yalmip.
%
%
%------------------------------------------------------------------
% This file is part of <a href="matlab:docsearch aroc">AROC</a>, a Toolbox for Automatic Reachset-
% Optimal Controller Synthesis developed at the Chair of Robotics,
% Artificial Intelligence and Embedded Systems,
% Technische Universitaet Muenchen.
%Yexact
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

    constraints = [];

    %% We start with the containment constraints
    G_final = Rtp{end}.Y.G;
    c_final = Rtp{end}.Y.c;
    ell = size(G_final,2);

    inbody.G = G_final;
    inbody.c = c_final;

    circumbody.G = terminalSet.G;
    circumbody.c = terminalSet.c;
    m = size(circumbody.G, 2);

    Gamma = sdpvar(m,ell+1,'full');
    GammaAbs = sdpvar(m,ell+1,'full');

    variables = {Gamma, GammaAbs};

    % The invariant set should contain the time-point reachable set
    constraints = [constraints circumbody.G * Gamma == [inbody.G inbody.c-circumbody.c]];
    for i = 1:m
        constraints = [constraints sum(GammaAbs(i,:)) <= 1];
    end
    
    % Constraints to ensure that Xabs represents the absolute values of Gamma
    constraints = [constraints Gamma(:) <= GammaAbs(:) -Gamma(:) <= GammaAbs(:)];

    % Next, we constrain the inputs
    for i = 1:length(utotal)

        product_U_A_U = optsInternal.stdConstraints.U.A * utotal{i}.G;

        product_U_A_U_abs = sdpvar(size(product_U_A_U,1), size(product_U_A_U,2),'full');
        variables{end+1} = product_U_A_U_abs;

        constraints = [constraints sum(product_U_A_U_abs,2) <= (optsInternal.stdConstraints.U.b - optsInternal.stdConstraints.U.A * utotal{i}.c)];
        constraints = [constraints product_U_A_U(:) <= product_U_A_U_abs(:) -product_U_A_U(:) <= product_U_A_U_abs(:)];
    end

    %% Finally, we constrain the time interval solutions
    % We start with the state
    if ~representsa(optsInternal.stdConstraints.X, 'fullspace')
        for i = 1:length(Rti)
            product_X_A_Rti = optsInternal.stdConstraints.X.A * Rti{i}.X.G;
    
            product_X_A_Rti_abs = sdpvar(size(product_X_A_Rti,1), size(product_X_A_Rti,2),'full');
            variables{end+1} = product_X_A_Rti_abs;
    
            constraints = [constraints sum(product_X_A_Rti_abs,2) <= optsInternal.stdConstraints.X.b - optsInternal.stdConstraints.X.A * Rti{i}.X.c];
            constraints = [constraints product_X_A_Rti(:) <= product_X_A_Rti_abs(:) -product_X_A_Rti(:) <= product_X_A_Rti_abs(:)];
        end
    end
end