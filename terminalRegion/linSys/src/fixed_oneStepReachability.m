function [Rtp, Rti, utotal] = fixed_oneStepReachability(Rtp_previous, optsInternal)
% FIXED_ONESTEPREACHABILITY - A variant of implicit_oneStepReachability,
%       where there are no loose variables (necessary for the 'fixed'
%       approach) and where U_X and U_V are set to 0. Also, we ignore Y and
%       measurement noises, and only compute the state reachable set.
%       Finally, we also take into account the equilibrium point.
%
% Syntax:
%       [Rtp, Rti, ctrl_variable, utotal, additional_variables, additional_constraints] = fixed_oneStepReachability(Rtp_previous, optsInternal)
%
% Description:
%       Computes one step of linear reachability analysis, using the
%       trivial controller.
%
% Input Arguments:
%       -Rtp_previous: Struct containing the variables describing
%                      the time-point solution from the previous step
%
%       -optsInternal: Struct containing internal parameters
%
% Output Arguments:
%       -Rtp:          Struct containing the variables describing
%                      the time-point solution
%       -Rti:          Struct containing the variables describing
%                      the time-interval solution
%
%       - utotal:      Struct containing the variables describing
%                      the input
%
% See Also:
%       implicit_oneStepReachability
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
% Copyright (c) 2025 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % More convenient
    reachability = optsInternal.reachability;
    sys = optsInternal.stdSys;
    dims = optsInternal.stdDynDims;

    utotal.G = optsInternal.K*Rtp_previous.X.G;
    utotal.G = [utotal.G sparse(dims.nu, dims.nv)];
    utotal.c = optsInternal.K*(Rtp_previous.X.c-optsInternal.xEq) + optsInternal.uEq;

    Xhom.G = (reachability.eAdt+reachability.Phi*sys.B*optsInternal.K)*Rtp_previous.X.G;
    Xhom.c = reachability.eAdt*Rtp_previous.X.c+reachability.Phi*sys.B*(optsInternal.K*(Rtp_previous.X.c-optsInternal.xEq)+optsInternal.uEq) + reachability.Phi*sys.chi;

    Rtp.X.G = [Xhom.G reachability.P_dist];
    Rtp.X.c = Xhom.c;

    % Extending
    utotal_hom.G = utotal.G;
    utotal.G = [utotal.G sparse(dims.nu, reachability.m_P_dist)];

    % Let us extract the homogeneous parts:
    m_X_previous = size(Rtp_previous.X.G, 2);
    m = size(Xhom.G, 2);
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
    [C_G, C_c] = multIM_zono(combined_interval, combined_matrix, combined_center);

    % We can now easily construct the time interval solution
    [comb_G, comb_c] = combZonos(H_0, H_1, H_0_c, H_1_c);
    Rti.X.G = [comb_G C_G optsInternal.reachability.P_dist];
    Rti.X.c = comb_c + C_c;
    
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

function [G, c] = multIM_zono(intervalMatrix, zonoGenerators, zonoCenter)
    % center and radius of interval matrix
    M_c = center(intervalMatrix);
    M_r = rad(intervalMatrix);

    zonoGenerators_abs = abs(zonoGenerators);
    zonoCenter_abs = abs(zonoCenter);
    
    % absolute value of zonotope center and generators
    Zabssum = sum([zonoCenter_abs zonoGenerators_abs],2);
    
    G = [M_c*zonoGenerators diag(M_r*Zabssum)];

    c = M_c * zonoCenter;
end



