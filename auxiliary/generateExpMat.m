function expMat = generateExpMat(varargin)
% GENERATEEXPMAT - generates an exponent matrix up to specified order
%
% Syntax:
%       expMat = GENERATEEXPMAT(orders)
%       expMat = GENERATEEXPMAT(n,maxorder)
%
% Description:
%       This function generates an exponent matrix up to the specified
%       order for a specified number of variables
%
% Input Arguments:
%
%       -orders:    vector of orders (dimension: number of variables n)
%       -n:         number of variables
%       -maxorder:  maximum order (sum of each individual maximum order)
%
% Output Arguments:
%       -expMat:    expMat for given order
%
% See Also:
%       -
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
% Copyright (c) 2021 Chair of Robotics, Artificial Intelligence and
%               Embedded Systems, TU Muenchen
%------------------------------------------------------------------

    if nargin==0
        error('either one or two arguments required');
    elseif nargin==1
        orders = varargin{1};
        n = length(orders);
    elseif nargin==2
        n = varargin{1};
        maxorder = varargin{2};
        orders = maxorder*ones(n,1);
    end

    % generate expMat with individual order of up to orders(ii)
    args = arrayfun(@(ii)(0:orders(ii)),(1:n)','UniformOutput',false);
    out = cell(1,n);
    [out{:}] = ndgrid(args{:});
    out = arrayfun(@(ii) reshape(out{ii},[numel(out{ii}),1]),1:n,'UniformOutput',false);
    expMat = [out{:}]';
    
    if nargin==2
        % remove columns of expMat for which sum(expMat,1)>maxorder
        expMat(:,sum(expMat,1)>maxorder) = [];
    end