function [Rtp, Rti, ctrl_variable, utotal, additional_variables, additional_constraints] = implicit_oneStepReachability(Rtp_previous, optsInternal)
% IMPLICIT_ONESTEPREACHABILITY - Computes one step of linear reachability
%       analysis, as used by the linSys algorithm
%
% Syntax:
%       [Rtp, Rti, ctrl_variable, utotal, additional_variables, additional_constraints] = implicit_oneStepReachability(Rtp_previous, optsInternal)
%
% Description:
%       Computes one step of linear reachability analysis, using the
%       implicit approach for the controller. This entire
%       algorithm is set up such that the reachability is computed
%       symbolically, which is why we couldn't use CORA directly here.
%
% Input Arguments:
%       -Rtp_previous: Struct containing the symbolic variables describing
%                      the time-point solution from the previous step
%
%       -optsInternal: Struct containing internal parameters
%
% Output Arguments:
%       -Rtp:          Struct containing the symbolic variables describing
%                      the time-point solution
%       -Rti:          Struct containing the symbolic variables describing
%                      the time-interval solution
%
%       - ctrl_variable: Struct containing the symbolic variables necessary
%                      to describe the controller
%
%       - utotal:      Struct containing the symbolic variables describing
%                      the input 
%
%       -additional_variables, additional_constraints:
%                      These are the auxilliary variables and constraints
%                      that are needed to build the optimization problem,
%                      but don't necessarily have an interpretation.
%
% See Also:
%       ---
%
% References:
%       * *[1] "Approximability of the Containment Problem for Zonotopes
%              and Ellipsotopes", A. Kulmburg et al., submited to TAC
%              in 2024
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
% Authors:      Adrian Kulmburg
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2023 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % More convenient
    reachability = optsInternal.reachability;
    sys = optsInternal.stdSys;
    dims = optsInternal.stdDynDims;


    ctrl_variable.U_X = sdpvar(dims.nu, size(optsInternal.postH,1), 'full');
    ctrl_variable.U_V = sdpvar(dims.nu, dims.nv, 'full');
    ctrl_variable.u_c = sdpvar(dims.nu, 1, 'full');

    additional_variables.U_X = ctrl_variable.U_X;
    additional_variables.U_V = ctrl_variable.U_V;
    additional_variables.u_c = ctrl_variable.u_c;

    m_y_previous = size(Rtp_previous.Y.G,2);

    utotal.G = optsInternal.K*Rtp_previous.Y.G + [ctrl_variable.U_X*optsInternal.postH ctrl_variable.U_V sparse(dims.nu, m_y_previous-size(optsInternal.postH,2)-dims.nv)];
    utotal.G = [utotal.G sparse(dims.nu, dims.nv)];
    utotal.c = optsInternal.K*Rtp_previous.Y.c + ctrl_variable.u_c;

    m = size(Rtp_previous.Y.G,2) + dims.nv;
    Xhom.G = sdpvar(dims.nx, m, 'full');
    Xhom.c = sdpvar(dims.nx, 1, 'full');

    additional_variables.Xhom_G = Xhom.G;
    additional_variables.Xhom_c = Xhom.c;

    Xhom.G = [(reachability.eAdt + reachability.Phi*sys.B*optsInternal.K)*Rtp_previous.Y.G sys.F] + reachability.Phi*sys.B*[ctrl_variable.U_X*optsInternal.postH ctrl_variable.U_V sparse(dims.nu,m-size(optsInternal.postH,2)-dims.nv)];
    Xhom.c = (reachability.eAdt + reachability.Phi*sys.B*optsInternal.K)*Rtp_previous.Y.c + reachability.Phi*sys.B*ctrl_variable.u_c+reachability.Phi*sys.chi - reachability.eAdt*sys.gamma;

    Rtp.X.G = [Xhom.G reachability.P_dist];
    Rtp.X.c = Xhom.c;

    Rtp.Yexact.G = Rtp.X.G;
    Rtp.Yexact.c = Rtp.X.c + sys.gamma;

    % Extending
    utotal_hom.G = utotal.G;
    utotal.G = [utotal.G sparse(dims.nu, reachability.m_P_dist)];

    Rtp.Y.G = [Rtp.Yexact.G sys.F];
    Rtp.Y.c = Rtp.Yexact.c;

    % Extending again
    utotal.G = [utotal.G sparse(dims.nu, dims.nv)];


    % Let us extract the homogeneous parts:
    m_X_previous = size(Rtp_previous.X.G, 2);
    H_0 = [Rtp_previous.X.G sparse(dims.nx, m-m_X_previous)]; % Padding with zero values to make sure H_0 and H_1 have the same sizes
    H_0_c = Rtp_previous.X.c;
    H_1 = Xhom.G;
    H_1_c = Xhom.c;

    % Deduce the part that was added by the constant-like parts
    u_const = sys.B * utotal_hom.G;
    u_const_c = sys.B * utotal.c + sys.chi;

    % Now, it's time for the time interval solution. We first compute the
    % curvature error:
    combined_interval = [reachability.F_interval reachability.G_interval];

    combined_matrix = [H_0; u_const];
    combined_center = [H_0_c; u_const_c];
    [C_G, C_c, additional_constraints, additional_variables_multIM] = multIM_zono(combined_interval, combined_matrix, combined_center);

    additional_variables.multIM = additional_variables_multIM;

    % We can now easily construct the time interval solution
    [comb_G, comb_c] = combZonos(H_0, H_1, H_0_c, H_1_c);
    Rti.X.G = [comb_G C_G optsInternal.reachability.P_dist];
    Rti.X.c = comb_c + C_c;

    Rti.Yexact.G = Rti.X.G;
    Rti.Yexact.c = Rti.X.c + sys.gamma;

    
    Rti.Y.G = [Rti.Yexact.G sys.F];
    Rti.Y.c = Rti.Yexact.c;
    
end

function [comb_G, comb_c] = combZonos(zonoGenerators1, zonoGenerators2, center1, center2)
    m1 = size(zonoGenerators1,2);
    m2 = size(zonoGenerators2,2);

    comb_c = 0.5 * (center1 + center2);

    if m1 == m2
        comb_G = [0.5 * (center1 - center2) 0.5 * (zonoGenerators1 + zonoGenerators2) 0.5 * (zonoGenerators1 - zonoGenerators2)];
    else
        error('Mismatch of number of generators. This should not happen, so something deeply wrong happened within the algorithm...')
    end
end

function [G, c, additional_constraints,additional_variables] = multIM_zono(intervalMatrix, zonoGenerators, zonoCenter)
    % center and radius of interval matrix
    M_c = center(intervalMatrix);
    M_r = rad(intervalMatrix);

    n = size(zonoGenerators,1);
    m = size(zonoGenerators,2);

    zonoGenerators_abs = sdpvar(n,m,'full');
    zonoCenter_abs = sdpvar(n,1,'full');

    additional_constraints = [zonoGenerators(:) <= zonoGenerators_abs(:) -zonoGenerators(:) <= zonoGenerators_abs(:)];
    

    additional_constraints = [additional_constraints zonoCenter <= zonoCenter_abs -zonoCenter <= zonoCenter_abs];
    
    % absolute value of zonotope center and generators
    Zabssum = sum([zonoCenter_abs zonoGenerators_abs],2);
    
    G = [M_c*zonoGenerators diag(M_r*Zabssum)];
    additional_variables = {zonoGenerators_abs zonoCenter_abs};

    c = M_c * zonoCenter;
end



