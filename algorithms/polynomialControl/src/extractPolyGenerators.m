function list = extractPolyGenerators(pZ_c,Opts)
% EXTRACTPOLYGENERATORS - extract polynomial generators from poly. zonotope
%
% Syntax:
%       list = EXTRACTPOLYGENERATORS(pZ,Opts)
%
% Description:
%       Computes the optimal control parameters by solving the optimization
%       problem (7) in [1].
%
% Input Arguments:
%
%       -pZ_c:  cell array of polynomial zonotopes
%       -Opts:  structure containing all options
%
% Output Arguments:
%
%       -list:  list storing the polynomial zonotopes for the polynomial
%               generators
%
% See Also:
%       computeCtrl
%
% References:
%       * *[1] Gassman et al. (2021)*, Verified Polynomial Controller 
%              Synthesis for Disturbed Nonlinear Systems
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
    w = Opts.extHorizon.weights;
    Q = Opts.Q;

    list = {};

    % loop over all time steps
    for i = 1:length(pZ_c)

        % get poly. zonotopes for single generators
        pZ_i = restoreId(pZ_c{i},Opts.idv);
        [~,pZ_gen] = partZonotope(noIndep(pZ_i),Opts.idv);

        % construct cost function
        for j = 1+Opts.refInput:length(pZ_gen)
            list{end+1} = w(i)*Q*pZ_gen{j};
        end
    end
end