function [R,auxTerm] = reachControlledLTISystem(Opts,R0,inpSets,Rpar,Rtran)
% REACHCONTROLLEDLTISYSTEM - compute reachable sets for controlled LTI
%                            system
%
% Syntax:
%       [R,auxTerm] = REACHCONTROLLEDLTISYSTEM(Opts,R0,inpSets,Rpar,Rtran)
%
% Description:
%       This function computes reachable sets for controlled linear
%       time-invariant systems. We consider additional terms required for
%       the multiplication of a zonotope and a interval matrix separately
%       to ensure convexity of the subsequent optimization problem, because
%       abs(abs(X)) terms are problematic.
%
% Input Arguments:
%
%       -Opts:              a structure containing following options
%
%           -.TpOrTi:       compute reachable sets at time point only
%                           (Opts.TpOrTi = 'Tp') or also the time interval 
%                           reachable sets (Opts.TpOrTi = 'Ti')
%           -.N:            number of time-steps until safe terminal set is
%                           reached starting in safe initial set
%           -.XKX:          feedback matrix for the extended system
%                           dynamics
%           -.taylor:       struct containing the propagation matrices for
%                           the linear system
%
%       -R0:                initial set (given by Z-matrix storing center
%                           and generators)
%       -inpSets:           multidimensional array storing the centers and 
%                           generators of the input correction zonotopes
%       -Rpar:              particluar solution due to uncertain inputs
%       -Rtrans:            particular solution due to constant inputs
%
% Output Arguments:
%
%       -R:                 cell-array containing overall reachable sets
%       -auxTerm:           cell-array containing auxiliary terms required
%                           for the constraints
%
% See Also:
%       computeTermRegZonoLinSys
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
% Authors:      Felix Gruber
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization
    auxTerm = cell(Opts.N,1);
    R.Tp = cell(Opts.N,1);
    R.Ti = cell(Opts.N,1);
    
    zeroMatX = zeros(Opts.nx,size(R0, 2));

    % compute reachable set
    RTp = [R0; zeros(Opts.nu,size(R0, 2))];

    for i = 1:Opts.N
        
        % time point reachable set
        Rinit = Opts.XKX*RTp;
        
        if ~isempty(inpSets)
            zeroMatR = zeros(Opts.nxPlusnu,size(Rinit,2)-size(R0,2));
            Rinit = Rinit + [[zeroMatX; inpSets(:,:,i)],zeroMatR];
        end
        
        if ~isempty(Opts.KV)
            Rinit(Opts.nx+1:end,1) = Rinit(Opts.nx+1:end,1) + Opts.KV(:,1);
            zeroMatV = zeros(Opts.nx,size(Opts.KV,2)-1);
            Rinit = [Rinit, [zeroMatV; Opts.KV(:,2:end)]];
        end
        
        Rhom_tp = addZonotopes(Opts.taylor.eAt*Rinit, Rtran);
        RTp = addZonotopes(Rhom_tp, Rpar);
        R.Tp{i} = RTp;

        % time interval reachable set
        if strcmp(Opts.TpOrTi, 'Ti')
            auxTerm{i} = auxiliaryTerm(Opts.taylor.FRadius, Rinit);
            FCenterTimesRinit = Opts.taylor.FCenter * Rinit;
            enc = encloseZonotopes(Rinit, Rhom_tp);
            RTi = addZonotopes(enc, addZonotopes(FCenterTimesRinit, Rpar));
            R.Ti{i} = RTi;
        end
    end
end


% Auxiliary Functions -----------------------------------------------------

function Z = addZonotopes(Z1,Z2)
% Minkowski addition of two zonotopes

    Z = [Z1(:,1) + Z2(:,1), Z1(:,2:end), Z2(:,2:end)]; 
end

function result = auxiliaryTerm(X, Y)
% compute auxiliary term required for multiplication of zonotope and 
% interval matrix (see "contSet\@zonotope\intervalMultiplication" in CORA)

    absY = abs(Y);
    XTimesSumAbsY = X * sum(absY, 2);
    result = diag(XTimesSumAbsY);
end

function Z = encloseZonotopes(Z1, Z2)
% convex hull of two zonotopes (see "contSet\@zonotope\enclose.m" in CORA)

    % number of generators
    m1 = size(Z1, 2);
    m2 = size(Z2, 2);

    % split zonotopes into corresponding parts
    Zcut = Z2(:, 1:m1);
    Zadd = Z2(:, (m1+1):m2);
    Zequal = Z1;

    % compute enclosing zonotope
    Z = [(Zcut+Zequal)/2, (Zcut-Zequal)/2, Zadd];
end