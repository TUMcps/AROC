function obj = getAbsErrArguments(obj,dynamics)
%GETABSERRARGUMENTS - identifies which states influence the abstraction
%       error.
%
% Syntax:
%       obj = getAbsErrArguments(obj,dynamics)
%
% Description:
%       To reduce the conservatism of the parameterized abstraction error
%       (over-)approxmations employed in [1,2], this function identifies
%       which states influence the abstraction error (or not).
%
% Input Arguments:
%       -obj:       updated instance of class
%                   computeTermRegNonlinSysOpt
%       -dynamics:  instance of class termRegNonlinSysDynamics
%
% Output Arguments:
%       - obj:          updated instance of class
%                       computeTermRegNonlinSysOpt
%
% See Also:
%       -
%
% References:
%       [1] L. Schäfer et al. "Scalable Computation of Robust Control 
%           Invariant Sets of Nonlinear Systems", in IEEE Transactions 
%           on Automatic Control, vol. 69, no. 2, pp. 755-770, 2024
%       [2] L. Schäfer and M. Althoff, "Computing Robust Control 
%           Invariant Sets of Nonlinear Systems Using Polynomial 
%           Controller Synthesis," American Control Conference, 2024, 
%           pp. 4162-4169. 
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
% Authors:      Lukas Schäfer
% Website:      <a href="http://aroc.in.tum.de">aroc.in.tum.de</a>
% Work Adress:  Technische Universitaet Muenchen
% Copyright (c) 2024 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------ 


% check if set of admissible states is compact (by definition, it suffices
% to check boundedness)
if isBounded(obj.H_X)
    
    % encloses set of admissible states by an interval
    I_X = interval(obj.H_X);
    
    % compute interval enclosure of augmented reachable set
    % (order reduction to ensure recursive feasibility and simplify
    % computation of Lagrange remainder approximations
    Zy = reduce(cartProd(I_X,obj.Z_U),'girard',1);
    
    if isa(obj,'computeTermRegNonlinSysPolyApproach')
        % set correct tensor files
        nlSys = setThirdOrderTensor(dynamics.sys,'int');
    
        % evaluate third-order tensor
        [T,ind] = nlSys.thirdOrderTensor(interval(Zy),interval(obj.W));
    
        % iterate over tensors to find terms with non-zero coefficients
        argAbsErr = [];
        for ii=1:length(ind)
            for jj=1:length(ind{ii})
                [col,row] = find(max(abs(T{ii,ind{ii}(jj)}.inf), ...
                    abs(T{ii,ind{ii}(jj)}.sup)));
                argAbsErr = union(argAbsErr,union(col,row),'sorted');
            end
        end
    elseif isa(obj,'computeTermRegNonlinSysLinApproach') 
        % bound abstraction error as proposed in [1, Proposition 1]
    
        % set correct tensor file
        nlSys = setHessian(dynamics.sys,'int');
    
        % evaluate second-order tensor
        H = nlSys.hessian(interval(Zy),interval(obj.W));
    
        % iterate over tensors to find terms with non-zero coefficients
        argAbsErr = [];
        for ii=1:length(H)
            [col,row] = find(max(abs(H{ii}.inf), abs(H{ii}.sup)));
            argAbsErr = union(argAbsErr,union(col,row),'sorted');
        end
    end
else
    % unbounded set of admissible states: for now consider every state
    % as a possible input argument
    argAbsErr = 1:obj.nx;
end

obj.argAbsErr = argAbsErr(argAbsErr <= obj.nx);

end