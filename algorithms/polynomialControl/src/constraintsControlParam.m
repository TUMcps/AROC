function [A,b] = constraintsControlParam(P)
% CONSTRAINTSCONTROLPARAM - constraints for the control parameter
%
% Syntax:
%       [A,b] = CONSTRAINTSCONTROLPARAM(P)
%
% Description:
%       Computes the linear inequality constraints A*[p;z] <= b for the
%       parameter p of the polynomial controller with auxiliary variable z
%
% Input Arguments:
%
%       -P:     struct storing the list with the splitted parameter
%               templates
%
% Output Arguments:
%
%       -A:     matrix for the inequality constraint A*[p;z] <= b 
%       -b:     offset vector for the inequality constraint A*[p;z] <= b
%
% See Also:
%       polynomialControl
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
% Authors:      Victor Gassmann
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2020 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    % initialization
    nu = P.nu;
    steps = P.steps;
    Nz_vec = cellfun(@(c)size(c,1),P.list);
    splits = length(P.list);
    m = nu*steps;
    Nz = sum(Nz_vec)*nu*steps;
    
    % put Pkernel into halfspace rep
    Alist1 = cell(length(P.list),1);
    Alist2 = cell(length(P.list),1);
    
    for i = 1:splits
    
       % set for scalar input => extend to multi-dim input + nr. of steps
       Atmp = repmat(P.list(i),m,1);
       Alist1{i} = blkdiag(Atmp{:});
    
       % all coefficients corresponding to same set added up <=1
       Alist2{i} = repelem(eye(m),1,Nz_vec(i));
    end
    
    A1 = vertcat(Alist1{:});
    A2 = blkdiag(Alist2{:});
    b2 = zeros(sum(Nz_vec)*nu*steps,1);   
    
    % construct final matrices:
    %   1. row: sum_i z_i <= 1 (for each splitted set)
    %   2. row: |g^(i)(p)| "=" z_i

    A = [zeros(size(A2,1),size(A1,2)),A2;
         [A1;-A1],-[eye(Nz);eye(Nz)]];

    b = [ones(splits*nu*steps,1);
         b2;-b2];
