function [variables, constraints] = implicit_stateInputContainment(optsInternal, containmentType, Rtp, Rti, utotal, preH, s)
% IMPLICIT_STATEINPUTCONTAINMENT - sets up the containment-based constraints on
%       the state and input for the containmentLinSys algorithm
%
% Syntax:
%       [variables, constraints] = implicit_stateInputContainment(optsInternal, containmentType, Rtp, Rti, utotal, preH, s)
%
% Input Arguments:
%       containmentType:   Either 'ellipsoid' or 'zonotope'
%
%       Rtp:               cell array of structs describing the time-point
%                          solutions
%
%       Rti:               cell array of structs describing the
%                          time-interval solutions
%
%       utotal:            cell array of structs describing the input
%
%       preH:              matrix multiplied to the left of Diag(s) (see
%                          setOptsInternal.m)
%
%       s:                 scaling factors stored as sdpvar variables
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
    c_start = Rtp{1}.X.c;
    c_final = Rtp{end}.Y.c;
    ell = size(G_final,2);

    inbody.G = G_final;
    inbody.c = c_final;

    Gamma = sdpvar(optsInternal.size_s,ell+1,'full');
    GammaAbs = sdpvar(optsInternal.size_s,ell+1,'full');

    variables = {Gamma, GammaAbs};

    % The invariant set should contain the time-point reachable set
    constraints = [constraints preH * Gamma == [inbody.G inbody.c-c_start-optsInternal.stdSys.gamma]];
    for i = 1:optsInternal.size_s
        if strcmp(containmentType, 'ellipsoid')
            constraints = [constraints sum(GammaAbs(i,:)) <= 1/sqrt(optsInternal.size_s) * s(i)];
        else
            constraints = [constraints sum(GammaAbs(i,:)) <= s(i)];
        end
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

    % Next, Yexact
    if ~representsa(optsInternal.stdConstraints.Yexact, 'fullspace')
        for i = 1:length(Rti)
            product_Yexact_A_Rti = optsInternal.stdConstraints.Yexact.A * Rti{i}.Yexact.G;
    
            product_Yexact_A_Rti_abs = sdpvar(size(product_Yexact_A_Rti,1), size(product_Yexact_A_Rti,2),'full');
            variables{end+1} = product_Yexact_A_Rti_abs;
    
            constraints = [constraints sum(product_Yexact_A_Rti_abs,2) <= optsInternal.stdConstraints.Yexact.b - optsInternal.stdConstraints.Yexact.A * Rti{i}.Yexact.c];
            constraints = [constraints product_Yexact_A_Rti(:) <= product_Yexact_A_Rti_abs(:) -product_Yexact_A_Rti(:) <= product_Yexact_A_Rti_abs(:)];
        end
    end

    % Finally, we deal with Y
    if ~representsa(optsInternal.stdConstraints.Y, 'fullspace')
        for i = 1:length(Rti)
            product_Y_A_Rti = optsInternal.stdConstraints.Y.A * Rti{i}.Y.G;
    
            product_Y_A_Rti_abs = sdpvar(size(product_Y_A_Rti,1), size(product_Y_A_Rti,2),'full');
            variables{end+1} = product_Y_A_Rti_abs;
    
            constraints = [constraints sum(product_Y_A_Rti_abs,2) <= optsInternal.stdConstraints.Y.b - optsInternal.stdConstraints.Y.A * Rti{i}.Y.c];
            constraints = [constraints product_Y_A_Rti(:) <= product_Y_A_Rti_abs(:) -product_Y_A_Rti(:) <= product_Y_A_Rti_abs(:)];
        end
    end
end